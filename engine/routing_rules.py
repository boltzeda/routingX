#!/usr/bin/env python3
"""Validation and resolution for board-specific per-net routing classes."""

from __future__ import annotations

import math


RULE_KEYS = (
    "trace_width", "clearance", "via_drill", "via_annular_ring",
    "via_track_width", "hole_clearance", "outline_clearance",
)


def _positive(value, label):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{label} must be numeric, got {value!r}") from None
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{label} must be finite and > 0, got {value!r}")
    return number


def normalize_routing_rules(drc, known_nets=None):
    """Return a normalized DRC dict with deterministic per-net classes.

    ``net_classes`` is an optional list of objects.  Every class requires a
    unique name and a non-empty ``nets`` list, and may override any RULE_KEYS.
    Values omitted by the class inherit the board-wide defaults.  A net may be
    assigned to only one class so router behaviour never depends on list order.
    """
    result = dict(drc or {})
    for key in RULE_KEYS:
        if result.get(key) is not None:
            result[key] = _positive(result[key], f"drc.{key}")

    raw_classes = result.get("net_classes") or []
    if not isinstance(raw_classes, list):
        raise ValueError("drc.net_classes must be a JSON list")
    known = {str(net) for net in known_nets} if known_nets is not None else None
    names = set()
    assigned = {}
    normalized = []
    for index, original in enumerate(raw_classes):
        if not isinstance(original, dict):
            raise ValueError(f"drc.net_classes[{index}] must be an object")
        item = dict(original)
        name = str(item.get("name") or "").strip()
        if not name:
            raise ValueError(f"drc.net_classes[{index}].name is required")
        if name == "Default" or name in names:
            raise ValueError(f"routing class name must be unique and not 'Default': {name!r}")
        names.add(name)
        nets = item.get("nets")
        if not isinstance(nets, list) or not nets:
            raise ValueError(f"routing class {name!r} requires a non-empty nets list")
        clean_nets = []
        for raw_net in nets:
            net = str(raw_net).strip()
            if not net:
                raise ValueError(f"routing class {name!r} contains an empty net name")
            if known is not None and net not in known:
                raise ValueError(f"routing class {name!r} names unknown net {net!r}")
            if net in assigned:
                raise ValueError(
                    f"net {net!r} is assigned to both {assigned[net]!r} and {name!r}"
                )
            assigned[net] = name
            clean_nets.append(net)
        normalized_item = {"name": name, "nets": clean_nets}
        for key in RULE_KEYS:
            if item.get(key) is not None:
                normalized_item[key] = _positive(
                    item[key], f"routing class {name}.{key}"
                )
        normalized.append(normalized_item)
    result["net_classes"] = normalized
    return result


def rules_by_net(drc):
    """Resolve every explicitly classified net to a complete rule dictionary."""
    normalized = normalize_routing_rules(drc)
    base = {key: normalized.get(key) for key in RULE_KEYS if normalized.get(key) is not None}
    resolved = {}
    for item in normalized.get("net_classes", []):
        rule = dict(base)
        rule.update({key: item[key] for key in RULE_KEYS if key in item})
        rule["class_name"] = item["name"]
        for net in item["nets"]:
            resolved[net] = dict(rule)
    return resolved


def rule_for_net(drc, net):
    base = {key: drc[key] for key in RULE_KEYS if drc.get(key) is not None}
    return {**base, **rules_by_net(drc).get(str(net), {})}


def minimum_board_rules(drc):
    """Smallest manufactured geometry allowed by any configured net class."""
    normalized = normalize_routing_rules(drc)
    all_rules = [rule_for_net(normalized, net)
                 for item in normalized.get("net_classes", []) for net in item["nets"]]
    all_rules.append({key: normalized[key] for key in RULE_KEYS if normalized.get(key) is not None})
    return {
        key: min(rule[key] for rule in all_rules if rule.get(key) is not None)
        for key in RULE_KEYS
        if any(rule.get(key) is not None for rule in all_rules)
    }
