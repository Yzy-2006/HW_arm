"""Portable review plans for the local LeRobot editor."""

import hashlib
import json


def normalize_ranges(ranges, length):
    """Inclusive frame ranges; merge overlaps and adjacent deletions."""
    if not isinstance(ranges, list):
        raise ValueError("Deletion ranges must be a list")
    checked = []
    for pair in ranges:
        if (not isinstance(pair, list) or len(pair) != 2
                or any(type(n) is not int for n in pair)
                or not 0 <= pair[0] <= pair[1] < length):
            raise ValueError(f"Invalid deletion range: {pair}; episode has {length} frames")
        checked.append(pair)
    merged = []
    for start, end in sorted(checked):
        if merged and start <= merged[-1][1] + 1:
            merged[-1][1] = max(end, merged[-1][1])
        else:
            merged.append([start, end])
    return merged


def dataset_fingerprint(root, progress=None):
    """Hash metadata, numeric data and videos, independently of the local path."""
    files = sorted(path for folder in ("meta", "data", "videos")
                   for path in (root / folder).rglob("*") if path.is_file())
    if not files:
        raise ValueError("Dataset has no files")
    total = sum(path.stat().st_size for path in files)
    done = 0
    digest = hashlib.sha256()
    for path in files:
        digest.update(json.dumps(path.relative_to(root).as_posix()).encode())
        digest.update(b"\0")
        file_hash = hashlib.sha256()
        with path.open("rb") as source:
            while block := source.read(4 * 1024 * 1024):
                file_hash.update(block)
                done += len(block)
                if progress:
                    progress(done, total)
        digest.update(file_hash.digest())
    return "sha256:" + digest.hexdigest()


def validate_plan(plan, fingerprint, episodes):
    if not isinstance(plan, dict) or type(plan.get("version")) is not int or plan["version"] != 2:
        raise ValueError("需要版本 2 的协作方案；旧版方案不含数据身份信息，请在原界面核对后重新保存")
    if not fingerprint or plan.get("fingerprint") != fingerprint:
        raise ValueError("方案与当前数据集内容不一致，不能合并")
    expected = [{"index": e["episode_index"], "length": e["length"]} for e in episodes]
    if plan.get("episodes") != expected:
        raise ValueError("方案的 Episode 编号或帧数不一致")
    assigned, records = plan.get("assigned"), plan.get("records")
    valid_ids = {e["episode_index"] for e in episodes}
    if (not isinstance(assigned, list) or any(type(i) is not int or i not in valid_ids for i in assigned)
            or len(set(assigned)) != len(assigned) or not isinstance(records, dict)
            or set(records) != {str(i) for i in assigned}):
        raise ValueError("方案分工范围与记录不匹配")
    checked = {}
    for i in sorted(assigned):
        record = records[str(i)]
        if (not isinstance(record, dict) or type(record.get("reviewed")) is not bool
                or not isinstance(record.get("reviewer"), str) or len(record["reviewer"]) > 200):
            raise ValueError(f"Episode {i} 的完成状态或负责人无效")
        if record["reviewed"] and not record["reviewer"].strip():
            raise ValueError(f"Episode {i} 缺少负责人")
        checked[str(i)] = {
            "deletions": normalize_ranges(record.get("deletions"), episodes[i]["length"]),
            "reviewed": record["reviewed"], "reviewer": record["reviewer"].strip(),
        }
    return {"version": 2, "fingerprint": fingerprint, "episodes": expected,
            "assigned": sorted(assigned), "records": checked}


def merge_plans(current, incoming, fingerprint, episodes, choices=None):
    current = validate_plan(current, fingerprint, episodes)
    incoming = validate_plan(incoming, fingerprint, episodes)
    overlap = sorted(set(current["assigned"]) & set(incoming["assigned"]))
    choices = choices or {}
    if not isinstance(choices, dict) or set(choices) - {str(i) for i in overlap}:
        raise ValueError("无效的冲突选择")
    conflicts = []
    for i in overlap:
        key = str(i)
        if key not in choices:
            conflicts.append({"episode": i, "current": current["records"][key],
                              "incoming": incoming["records"][key]})
        elif choices[key] not in ("current", "incoming"):
            raise ValueError("请选择保留当前记录或导入记录")
    if conflicts:
        return {"conflicts": conflicts, "plan": None}
    records = current["records"].copy()
    for key, record in incoming["records"].items():
        if key not in records or choices.get(key) == "incoming":
            records[key] = record
    current.update(assigned=sorted(map(int, records)), records=records)
    return {"conflicts": [], "plan": current}


def require_complete(plan, fingerprint, episodes):
    plan = validate_plan(plan, fingerprint, episodes)
    missing = [e["episode_index"] for e in episodes
               if not plan["records"].get(str(e["episode_index"]), {}).get("reviewed")]
    if missing:
        raise ValueError("尚未完成的 Episode：" + ", ".join(map(str, missing)))
    return plan
