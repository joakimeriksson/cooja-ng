#!/usr/bin/env python3
"""
Convert Cooja .csc simulation files to csim JSON config format.

Parses the XML, extracts nodes/positions/radio medium, and translates
the JavaScript test script into csim JSON test steps/actions.

Usage:
  python3 tools/csc2json.py test.csc --contiki /path/to/contiki-ng \\
      --firmware-dir firmware/cc2538dk -o out.json
  python3 tools/csc2json.py test.csc --list-firmware
"""

import argparse
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from html import unescape


def parse_csc(csc_path):
    """Parse a .csc XML file, return the ElementTree root."""
    tree = ET.parse(csc_path)
    return tree.getroot()


def extract_radio_medium(sim_elem):
    """Extract radio medium settings from <radiomedium> element."""
    rm = sim_elem.find("radiomedium")
    if rm is None:
        return {}

    text = (rm.text or "").strip()
    is_udgm = "UDGM" in text

    result = {}
    if is_udgm:
        result["type"] = "udgm"

    for tag, key in [
        ("transmitting_range", "tx_range"),
        ("interference_range", "interference_range"),
        ("success_ratio_tx", "success_ratio_tx"),
        ("success_ratio_rx", "success_ratio_rx"),
    ]:
        elem = rm.find(tag)
        if elem is not None and elem.text:
            result[key] = float(elem.text.strip())

    return result


def extract_mote_types(sim_elem, csc_dir):
    """Extract mote type definitions: maps description to source file and build info."""
    mote_types = {}
    for mt in sim_elem.findall("motetype"):
        desc_elem = mt.find("description")
        source_elem = mt.find("source")
        commands_elem = mt.find("commands")
        desc = desc_elem.text.strip() if desc_elem is not None and desc_elem.text else None
        source = source_elem.text.strip() if source_elem is not None and source_elem.text else None
        commands = commands_elem.text if commands_elem is not None else None

        if source:
            source = source.replace("[CONFIG_DIR]", csc_dir)

        # Parse build info from <commands>
        build = parse_build_commands(commands)

        # Collect motes under this type
        motes = []
        for mote_elem in mt.findall("mote"):
            mote_info = {}
            for ic in mote_elem.findall("interface_config"):
                ic_text = (ic.text or "").strip()
                if "Position" in ic_text:
                    pos = ic.find("pos")
                    if pos is not None:
                        mote_info["x"] = float(pos.get("x", "0"))
                        mote_info["y"] = float(pos.get("y", "0"))
                elif "MoteID" in ic_text:
                    id_elem = ic.find("id")
                    if id_elem is not None and id_elem.text:
                        mote_info["id"] = int(id_elem.text.strip())
            motes.append(mote_info)

        if desc:
            mote_types[desc] = {"source": source, "motes": motes, "build": build}
        # Also index by ordinal (for getMoteTypes()[index] references)

    return mote_types


def parse_build_commands(commands_text):
    """Parse Contiki-NG <commands> element to extract build info.

    Extracts TARGET, BOARD, DEFINES, and other KEY=VALUE make arguments
    from the build command (the non-clean line).

    Returns dict with keys: target, board, make_args (list of extra KEY=VALUE).
    """
    if not commands_text:
        return None

    build = {}
    make_args = []

    # Find the actual build line (not the clean line)
    for line in commands_text.strip().split("\n"):
        line = line.strip()
        if not line or "clean" in line:
            continue

        # Parse KEY=VALUE tokens from the make command
        for token in line.split():
            if "=" not in token:
                continue
            key, val = token.split("=", 1)
            if key == "TARGET":
                build["target"] = val
            elif key == "BOARD":
                build["board"] = val
            elif key in ("$(MAKE)", "CONTIKI", "WERROR"):
                continue  # skip make itself and contiki path
            elif key.startswith("-"):
                continue  # skip flags like -j$(CPUS)
            else:
                make_args.append(f"{key}={val}")

    if make_args:
        build["make_args"] = make_args

    return build if build else None


def source_to_firmware_name(source_path):
    """Convert a source .c path to firmware name (strip .c, get basename)."""
    if not source_path:
        return None
    basename = os.path.basename(source_path)
    if basename.endswith(".c"):
        basename = basename[:-2]
    return basename


def extract_nodes(sim_elem, csc_dir, firmware_dir):
    """Extract all nodes with positions, IDs, firmware paths, and build info."""
    mote_types = extract_mote_types(sim_elem, csc_dir)
    nodes = []

    # Iterate mote types in order, collecting motes
    for mt in sim_elem.findall("motetype"):
        desc_elem = mt.find("description")
        source_elem = mt.find("source")
        commands_elem = mt.find("commands")
        desc = desc_elem.text.strip() if desc_elem is not None and desc_elem.text else ""
        source = source_elem.text.strip() if source_elem is not None and source_elem.text else None
        commands = commands_elem.text if commands_elem is not None else None

        if source:
            source = source.replace("[CONFIG_DIR]", csc_dir)

        # Parse build info to get target for extension detection
        build = parse_build_commands(commands)
        build_target = build.get("target") if build else None

        fw_name = source_to_firmware_name(source)
        if fw_name and firmware_dir:
            # Generate variant name from DEFINES (e.g., "intermediate-mpl")
            fw_variant = fw_name
            if build and build.get("make_args"):
                for arg in build["make_args"]:
                    if arg.startswith("DEFINES="):
                        defines = arg[8:]
                        for d in defines.split(","):
                            if "=" in d:
                                val = d.split("=")[-1]
                                short = val.split("_")[-1].lower() if "_" in val else val.lower()
                                fw_variant = f"{fw_name}-{short}"
                                break

            # Auto-detect extension: try variant name first, then base name
            fw_path = None
            for name_candidate in ([fw_variant, fw_name] if fw_variant != fw_name else [fw_name]):
                if build_target:
                    ext = "." + build_target
                    candidate = os.path.join(firmware_dir, name_candidate + ext)
                    if os.path.exists(candidate):
                        fw_path = candidate
                        break
                for ext in (".cooja", ".cc2538dk", ".sky"):
                    candidate = os.path.join(firmware_dir, name_candidate + ext)
                    if os.path.exists(candidate):
                        fw_path = candidate
                        break
                if fw_path:
                    break
            if fw_path is None:
                # Fall back to variant name with target extension
                fallback_ext = "." + build_target if build_target else ".cc2538dk"
                fw_path = os.path.join(firmware_dir, fw_variant + fallback_ext)
        elif fw_name:
            fallback_ext = "." + build_target if build_target else ".cc2538dk"
            fw_path = fw_name + fallback_ext
        else:
            fw_path = "unknown.cc2538dk"

        for mote_elem in mt.findall("mote"):
            node = {"firmware": fw_path}

            # Add build info if present
            if build:
                node["build"] = build
                # Also store source dir for building
                if source:
                    node["build"]["source_dir"] = os.path.dirname(source)

            for ic in mote_elem.findall("interface_config"):
                ic_text = (ic.text or "").strip()
                if "Position" in ic_text:
                    pos = ic.find("pos")
                    if pos is not None:
                        node["x"] = round(float(pos.get("x", "0")), 2)
                        node["y"] = round(float(pos.get("y", "0")), 2)
                elif "MoteID" in ic_text:
                    id_elem = ic.find("id")
                    if id_elem is not None and id_elem.text:
                        node["id"] = int(id_elem.text.strip())
            nodes.append(node)

    # Sort by ID if all have IDs
    if all("id" in n for n in nodes):
        nodes.sort(key=lambda n: n["id"])

    return nodes


def extract_startup_delay(sim_elem):
    """Extract motedelay_us and convert to ms."""
    elem = sim_elem.find("motedelay_us")
    if elem is not None and elem.text:
        return int(elem.text.strip()) // 1000
    return 1000  # default 1s


def extract_seed(sim_elem):
    """Extract random seed."""
    elem = sim_elem.find("randomseed")
    if elem is not None and elem.text:
        text = elem.text.strip()
        if text.isdigit():
            return int(text)
    return None


def extract_script(root, csc_dir):
    """Extract the test script from ScriptRunner plugin."""
    for plugin in root.findall("plugin"):
        plugin_text = (plugin.text or "").strip()
        if "ScriptRunner" not in plugin_text:
            continue

        pc = plugin.find("plugin_config")
        if pc is None:
            continue

        # Check for inline script
        script_elem = pc.find("script")
        if script_elem is not None and script_elem.text:
            return unescape(script_elem.text)

        # Check for external script file
        scriptfile_elem = pc.find("scriptfile")
        if scriptfile_elem is not None and scriptfile_elem.text:
            sf_path = scriptfile_elem.text.strip()
            sf_path = sf_path.replace("[CONFIG_DIR]", csc_dir)
            if os.path.exists(sf_path):
                with open(sf_path, "r") as f:
                    return f.read()
            else:
                print(f"WARNING: script file not found: {sf_path}", file=sys.stderr)
                return None

    return None


def clean_script(script):
    """Clean up \r\n and HTML entities in script text."""
    if not script:
        return ""
    # Normalize line endings
    script = script.replace("\r\n", "\n").replace("\r", "\n")
    # Unescape any remaining HTML entities
    script = unescape(script)
    return script


def detect_unsupported_features(script):
    """Detect features that can't be converted."""
    features = []
    if re.search(r'generateMote\s*\(', script):
        # generateMote is only unsupported when NOT part of a remove/add pattern
        # Check if all generateMote calls are in addMote handlers
        if not re.search(r'sim\.addMote\s*\(', script):
            features.append("generateMote")
    if re.search(r'java\.util\.Random', script):
        features.append("java.util.Random")
    return features


def parse_timeout(script):
    """Parse TIMEOUT(ms) or TIMEOUT(ms, callback).
    Returns (timeout_ms, timeout_is_success, fail_on_patterns, validators)."""
    validators = []

    # TIMEOUT(ms, if(condition) { log.testOK(); })
    m = re.search(
        r'TIMEOUT\s*\(\s*(\d+)\s*,\s*if\s*\(([^)]+)\)\s*\{\s*log\.testOK\(\)\s*;?\s*\}',
        script
    )
    if m:
        timeout_ms = int(m.group(1))
        condition = m.group(2)
        validators = _extract_validators(condition, script)
        return timeout_ms, True, [], validators

    # TIMEOUT(ms, log.testFailed())
    m = re.search(r'TIMEOUT\s*\(\s*(\d+)\s*,\s*log\.testFailed\(\)', script)
    if m:
        return int(m.group(1)), False, [], []

    # Simple TIMEOUT(ms)
    m = re.search(r'TIMEOUT\s*\(\s*(\d+)\s*\)', script)
    if m:
        return int(m.group(1)), False, [], []

    return None, False, [], []


def _extract_validators(condition, script):
    """Extract validators from a TIMEOUT condition expression.
    Detects patterns like:
      seenMsgs > 15           -> min_count = 16
      lastMsg != -1           -> min_count = 1  (at least one Data msg)
      lastMsg >= 62           -> min_count = 63
      lostMsgs == 0           -> (ignored, can't model with counters)
    The pattern ("Data") is detected from the script body."""
    validators = []

    # Detect the counting pattern from the script body
    pattern = _detect_counted_pattern(script)
    if not pattern:
        return validators

    # seenMsgs > N  ->  min_count = N+1
    m = re.search(r'seenMsgs\s*>\s*(\d+)', condition)
    if m:
        validators.append({
            "pattern": pattern,
            "min_count": int(m.group(1)) + 1,
            "node": -1
        })
        return validators

    # seenMsgs >= N  ->  min_count = N
    m = re.search(r'seenMsgs\s*>=\s*(\d+)', condition)
    if m:
        validators.append({
            "pattern": pattern,
            "min_count": int(m.group(1)),
            "node": -1
        })
        return validators

    # lastMsg >= N  ->  min_count = N+1 (at least N+1 messages seen)
    m = re.search(r'lastMsg\s*>=\s*(\d+)', condition)
    if m:
        validators.append({
            "pattern": pattern,
            "min_count": int(m.group(1)) + 1,
            "node": -1
        })
        return validators

    # lastMsg != -1  ->  min_count = 1 (at least one message)
    if re.search(r'lastMsg\s*!=\s*-1', condition):
        validators.append({
            "pattern": pattern,
            "min_count": 1,
            "node": -1
        })
        return validators

    # packetsReceived.length > N  ->  min_count = N+1
    m = re.search(r'packetsReceived\.length\s*>\s*(\d+)', condition)
    if m:
        validators.append({
            "pattern": pattern,
            "min_count": int(m.group(1)) + 1,
            "node": -1
        })
        return validators

    # packetsReceived.length >= N  ->  min_count = N
    m = re.search(r'packetsReceived\.length\s*>=\s*(\d+)', condition)
    if m:
        validators.append({
            "pattern": pattern,
            "min_count": int(m.group(1)),
            "node": -1
        })
        return validators

    return validators


def _detect_counted_pattern(script):
    """Detect which message pattern is being counted in the script.
    Looks for seenMsgs++ or lastMsg assignment in the same block as
    msg.startsWith/contains check."""
    # Find all msg.startsWith("X") or msg.contains("X") blocks
    for m in re.finditer(
        r'msg\.(?:startsWith|contains)\s*\(\s*["\']([^"\']+)["\']\s*\)',
        script
    ):
        pattern = m.group(1)
        # Get the block between the { } of this else-if branch
        rest = script[m.end():]
        block = ""
        depth = 0
        started = False
        for i, ch in enumerate(rest):
            if ch == '{':
                if not started:
                    started = True
                    depth = 1
                    continue
                depth += 1
            elif ch == '}':
                if started:
                    depth -= 1
                    if depth == 0:
                        block = rest[:i]
                        break
            if started:
                block += ch
            if i > 2000:  # safety limit
                break
        if re.search(r'seenMsgs\s*\+\+|lastMsg\s*=|packetsReceived\.push', block):
            return pattern

    return None


def parse_wait_until_sequence(script):
    """Parse a sequence of WAIT_UNTIL statements into test steps."""
    steps = []
    # Match WAIT_UNTIL(condition)
    for m in re.finditer(r'WAIT_UNTIL\s*\(\s*(.+?)\s*\)\s*;', script):
        cond = m.group(1)
        step = parse_wait_condition(cond)
        if step:
            steps.append(step)
    return steps


def parse_wait_condition(cond):
    """Parse a WAIT_UNTIL condition into a test step dict."""
    step = {}

    # Check for node ID filter: id == N && ...
    node_m = re.match(r'id\s*==\s*(\d+)\s*&&\s*(.*)', cond)
    if node_m:
        step["node"] = int(node_m.group(1))
        cond = node_m.group(2).strip()

    # msg.contains("pattern")
    contains_m = re.search(r'msg\.contains\s*\(\s*["\']([^"\']+)["\']\s*\)', cond)
    if contains_m:
        step["wait"] = contains_m.group(1)
        return step

    # msg.equals("pattern")
    equals_m = re.search(r'msg\.equals\s*\(\s*["\']([^"\']+)["\']\s*\)', cond)
    if equals_m:
        step["wait"] = equals_m.group(1)
        return step

    # msg.startsWith("pattern")
    starts_m = re.search(r'msg\.startsWith\s*\(\s*["\']([^"\']+)["\']\s*\)', cond)
    if starts_m:
        step["wait"] = starts_m.group(1)
        return step

    return None


def parse_fail_conditions(script):
    """Extract fail_on patterns from the script."""
    patterns = []

    # if(msg.contains("X")) { log.testFailed() }
    for m in re.finditer(
        r'if\s*\(\s*msg\.contains\s*\(\s*["\']([^"\']+)["\']\s*\)\s*\)\s*\{?\s*'
        r'(?:log\.testFailed\(\)|failed\s*=\s*true)',
        script
    ):
        patterns.append(m.group(1))

    # if(msg.indexOf("X") != -1 ...) { log.testFailed() }
    for m in re.finditer(
        r'if\s*\(\s*msg\.indexOf\s*\(\s*["\']([^"\']+)["\']\s*\)\s*!=\s*-1[^)]*\)\s*\{[^}]*'
        r'log\.testFailed\(\)',
        script
    ):
        patterns.append(m.group(1))

    return patterns


def parse_yield_loop(script):
    """Parse a YIELD() loop looking for simple message matching patterns.
    Handles patterns like:
      while(true) { YIELD(); if(msg.contains("X")) { log.testOK(); } }
      while(true) { YIELD(); ... if(msg.contains("X")) break; } log.testOK();
    """
    steps = []

    # Look for msg.contains/equals/startsWith -> testOK or break patterns
    for m in re.finditer(
        r'if\s*\(\s*msg\.(contains|equals|startsWith)\s*\(\s*["\']([^"\']+)["\']\s*\)',
        script
    ):
        pattern = m.group(2)
        # Skip negated conditions (== false, != true)
        before_if = script[max(0, m.start()-5):m.start()]
        after_match = script[m.end():m.end()+30]
        if '== false' in after_match or '!= true' in after_match:
            continue
        # Skip conditions that lead to fail/error tracking, not success
        after = script[m.end():m.end()+200]
        leads_to_ok = False
        # Direct testOK within close range (same block, before any other if)
        if 'testOK' in after:
            ok_pos = after.find('testOK')
            fail_pos = after.find('testFailed')
            failed_var = after.find('failed')
            next_if = after.find('if(')
            next_if2 = after.find('if (')
            next_if_pos = min(p for p in [next_if, next_if2, ok_pos + 1] if p >= 0)
            if ok_pos < next_if_pos and \
               (fail_pos < 0 or fail_pos > ok_pos) and \
               (failed_var < 0 or failed_var > ok_pos):
                leads_to_ok = True
        # break in the immediate block (not in a later if statement)
        if not leads_to_ok:
            # Only match break within the first { } block
            imm_block = re.match(r'[^{]*\{([^}]*)\}', after)
            if imm_block and 'break' in imm_block.group(1):
                rest = script[m.end():]
                if re.search(r'}\s*\n\s*(?:if\s*\([^)]*\)\s*\{?\s*log\.testFailed[^}]*\}?\s*\n\s*)?log\.testOK', rest):
                    leads_to_ok = True
        if leads_to_ok:
            steps.append({"wait": pattern})

    return steps


def parse_counter_loop(script):
    """Parse counter-based loops like counting N matches of a pattern."""
    steps = []

    # Pattern: check for "DONE" message → break/testOK
    if re.search(r'msg\.contains\s*\(\s*["\']DONE["\']\s*\)', script):
        # Count how many motes need to say DONE
        m = re.search(r'done\s*(?:>=?|==)\s*(?:sim\.getMotes\(\)\.length|(\d+))', script)
        if m:
            count = int(m.group(1)) if m.group(1) else None
            step = {"wait": "DONE"}
            if count:
                step["count"] = count
            steps.append(step)
        else:
            # Simple DONE → break → testOK
            steps.append({"wait": "DONE"})

    return steps


def _resolve_script_vars(script):
    """Extract simple variable declarations: var name = number_or_string."""
    vars = {}
    for m in re.finditer(r'var\s+(\w+)\s*=\s*(\d+)\s*;', script):
        vars[m.group(1)] = int(m.group(2))
    return vars


def _resolve_value(expr, vars):
    """Resolve a numeric expression (literal or variable name) to int."""
    expr = expr.strip()
    if expr.isdigit():
        return int(expr)
    if expr in vars:
        return vars[expr]
    return None


def _resolve_string(expr, vars):
    """Resolve a string expression like '"text" + var' or '"text"'."""
    expr = expr.strip()
    # Simple string literal
    m = re.match(r'^["\']([^"\']*)["\']$', expr)
    if m:
        return m.group(1)
    # String concatenation: "text" + var
    parts = re.split(r'\s*\+\s*', expr)
    result = ""
    for p in parts:
        p = p.strip()
        sm = re.match(r'^["\']([^"\']*)["\']$', p)
        if sm:
            result += sm.group(1)
        elif p in vars:
            result += str(vars[p])
        else:
            return None  # can't resolve
    return result


def parse_generate_msg(script):
    """Parse GENERATE_MSG(timestamp, "label") and MY_GENERATE_MSG calls.
    Returns list of (timestamp_ms, label)."""
    msgs = []
    vars = _resolve_script_vars(script)

    # Standard GENERATE_MSG(timestamp, "label")
    for m in re.finditer(
        r'(?<!MY_)GENERATE_MSG\s*\(\s*(\d+)\s*,\s*["\']([^"\']+)["\']\s*\)', script
    ):
        timestamp_ms = int(m.group(1))
        label = m.group(2)
        msgs.append((timestamp_ms, label))

    # MY_GENERATE_MSG with accumulated time:
    # function MY_GENERATE_MSG(wait, string) { last_msg += wait; GENERATE_MSG(last_msg, string); }
    if re.search(r'function\s+MY_GENERATE_MSG', script):
        accumulated = 0
        for m in re.finditer(
            r'MY_GENERATE_MSG\s*\(\s*([^,]+?)\s*,\s*([^)]+?)\s*\)', script
        ):
            wait_expr = m.group(1).strip()
            label_expr = m.group(2).strip()

            wait_val = _resolve_value(wait_expr, vars)
            label_val = _resolve_string(label_expr, vars)

            if wait_val is not None:
                accumulated += wait_val
            if label_val is not None:
                msgs.append((accumulated, label_val))

    return msgs


def parse_setcoordinates(script, gen_msgs):
    """Parse setCoordinates patterns triggered by GENERATE_MSG labels.
    Returns list of action dicts."""
    actions = []

    # Look for patterns:
    #   if(msg.equals("label")) { ... setCoordinates(x, y, z) }
    # or place(id, x, y) function calls triggered by msg events
    for m in re.finditer(
        r'(?:sim\.getMoteWithID|node)\s*\(\s*(\d+)\s*\)[^;]*?'
        r'setCoordinates\s*\(\s*([^,]+)\s*,\s*([^,]+)\s*,\s*[^)]+\)',
        script
    ):
        node_id = int(m.group(1))
        try:
            x = float(m.group(2))
            y = float(m.group(3))
            actions.append({"type": "move", "node": node_id, "x": x, "y": y})
        except ValueError:
            pass  # Dynamic coordinates, can't convert statically

    return actions


def _find_handler_block(script, label):
    """Find handler code block(s) matching a GENERATE_MSG label.
    Tries exact match first, then startsWith prefix match.
    Returns list of (handler_start, block_text) tuples."""
    results = []

    # Try exact match: msg.equals("label")
    for m in re.finditer(
        r'msg\.equals\s*\(\s*["\']' + re.escape(label) + r'["\']\s*\)',
        script
    ):
        start = m.start()
        block = script[start:start + 500]
        results.append((start, block))

    # Try startsWith: msg.startsWith("prefix") where label starts with prefix
    for m in re.finditer(
        r'msg\.startsWith\s*\(\s*["\']([^"\']+)["\']\s*\)',
        script
    ):
        prefix = m.group(1)
        if label.startswith(prefix):
            start = m.start()
            block = script[start:start + 500]
            results.append((start, block))

    # Try contains: msg.contains("substring") where label contains substring
    for m in re.finditer(
        r'msg\.contains\s*\(\s*["\']([^"\']+)["\']\s*\)',
        script
    ):
        substring = m.group(1)
        if substring in label:
            start = m.start()
            block = script[start:start + 500]
            results.append((start, block))

    return results


def _extract_node_id_from_block(block, script):
    """Extract node ID from a handler block, trying literal then variable."""
    # Try literal getMoteWithID(N) or setMoteID(N)
    for pattern in [r'getMoteWithID\s*\(\s*(\d+)\s*\)',
                    r'setMoteID\s*\(\s*(\d+)\s*\)']:
        id_m = re.search(pattern, block)
        if id_m:
            return int(id_m.group(1))

    # Try variable-based: getMoteWithID(var) or setMoteID(var)
    for pattern in [r'getMoteWithID\s*\(\s*(\w+)\s*\)',
                    r'setMoteID\s*\(\s*(\w+)\s*\)']:
        id_m = re.search(pattern, block)
        if id_m:
            var_name = id_m.group(1)
            if var_name.isdigit():
                continue
            var_m = re.search(
                r'var\s+' + re.escape(var_name) + r'\s*=\s*(\d+)',
                script
            )
            if var_m:
                return int(var_m.group(1))

    return None


def parse_remove_add_actions(script, gen_msgs):
    """Parse removeMote/addMote patterns triggered by GENERATE_MSG labels.
    Returns list of action dicts with type 'remove' or 'add'."""
    actions = []

    if not re.search(r'sim\.removeMote\s*\(', script) and \
       not re.search(r'sim\.addMote\s*\(', script):
        return actions

    for timestamp_ms, label in gen_msgs:
        handlers = _find_handler_block(script, label)
        for _, block in handlers:
            if re.search(r'sim\.removeMote\s*\(', block):
                node_id = _extract_node_id_from_block(block, script)
                if node_id is not None:
                    actions.append({
                        "type": "remove", "node": node_id,
                        "at_ms": timestamp_ms
                    })
                    break
            elif re.search(r'sim\.addMote\s*\(', block):
                node_id = _extract_node_id_from_block(block, script)
                if node_id is not None:
                    actions.append({
                        "type": "add", "node": node_id,
                        "at_ms": timestamp_ms
                    })
                    break

    return actions


def parse_write_calls(script, gen_msgs):
    """Parse write(mote, "command") patterns triggered by GENERATE_MSG labels.
    Matches handler blocks to extract node ID, data, and timing.
    Returns list of action dicts."""
    actions = []

    if not re.search(r'write\s*\(', script):
        return actions

    for timestamp_ms, label in gen_msgs:
        handlers = _find_handler_block(script, label)
        for _, block in handlers:
            # Match write(m, "data") or write(motes[i], "data") patterns
            wm = re.search(
                r'write\s*\(\s*(\w+(?:\[\w+\])?)\s*,\s*["\']([^"\']+)["\']\s*\)',
                block
            )
            if not wm:
                continue

            data = wm.group(2)
            if not data.endswith("\n"):
                data += "\n"

            var_name = wm.group(1)
            # Check if it's write(motes[i], ...) — broadcast to all nodes
            if re.match(r'motes\[', var_name) or var_name == 'motes':
                actions.append({
                    "type": "send_all",
                    "data": data, "at_ms": timestamp_ms
                })
                break

            # Find node ID: look for getMoteWithID(N) assignment to var
            node_id = None
            # Same block: m = sim.getMoteWithID(N) ... write(m, ...)
            id_m = re.search(
                r'getMoteWithID\s*\(\s*(\d+)\s*\)', block
            )
            if id_m:
                node_id = int(id_m.group(1))
            else:
                # Variable-based: getMoteWithID(var)
                id_m = re.search(
                    r'getMoteWithID\s*\(\s*(\w+)\s*\)', block
                )
                if id_m:
                    vn = id_m.group(1)
                    var_m = re.search(
                        r'var\s+' + re.escape(vn) + r'\s*=\s*(\d+)',
                        script
                    )
                    if var_m:
                        node_id = int(var_m.group(1))

            if node_id is not None:
                actions.append({
                    "type": "send", "node": node_id,
                    "data": data, "at_ms": timestamp_ms
                })
                break

    return actions


def translate_script(script, nodes):
    """Translate a Cooja test script into csim JSON test config.
    Returns (test_config_dict, unsupported_features_list, warnings_list)."""
    script = clean_script(script)
    warnings = []
    unsupported = detect_unsupported_features(script)

    test = {}
    timeout_ms, timeout_is_success, _, validators = parse_timeout(script)
    if timeout_ms:
        test["timeout_ms"] = timeout_ms
    if timeout_is_success:
        test["timeout_is_success"] = True
    if validators:
        test["validators"] = validators

    # Extract fail_on patterns
    fail_on = parse_fail_conditions(script)
    if fail_on:
        test["fail_on"] = fail_on

    # Try to extract test steps
    steps = []

    # Method 1: WAIT_UNTIL sequence
    wait_steps = parse_wait_until_sequence(script)
    if wait_steps:
        steps.extend(wait_steps)

    # Method 2: YIELD loop with testOK on message match
    if not steps:
        yield_steps = parse_yield_loop(script)
        if yield_steps:
            steps.extend(yield_steps)

    # Method 3: Counter-based loops
    if not steps:
        counter_steps = parse_counter_loop(script)
        if counter_steps:
            steps.extend(counter_steps)

    # Method 4: For timeout_is_success tests with no explicit steps,
    # the test succeeds if it runs to timeout without hitting fail_on.
    # No steps needed — just fail_on + timeout_is_success.

    # Parse GENERATE_MSG for timed actions
    gen_msgs = parse_generate_msg(script)

    # Filter out steps that match GENERATE_MSG labels (script-internal messages,
    # not node output). If a filtered step was the termination condition,
    # convert to timeout_is_success at that timestamp.
    if steps and gen_msgs:
        gen_labels = {label for _, label in gen_msgs}
        filtered = [s for s in steps if s.get("wait") not in gen_labels]
        removed = [s for s in steps if s.get("wait") in gen_labels]
        if removed and not filtered:
            # All steps were script-internal — use timeout_is_success
            # with the latest matching GENERATE_MSG timestamp
            for s in removed:
                for ts, label in gen_msgs:
                    if label == s.get("wait"):
                        if not timeout_ms or ts > timeout_ms:
                            timeout_ms = ts + 5000  # small margin
                            test["timeout_ms"] = timeout_ms
            test["timeout_is_success"] = True
            timeout_is_success = True
        steps = filtered

    if steps:
        test["steps"] = steps

    # If no explicit TIMEOUT, derive from last GENERATE_MSG timestamp + margin
    if not timeout_ms and gen_msgs:
        last_ts = max(ts for ts, _ in gen_msgs)
        if last_ts > 0:
            timeout_ms = last_ts + 30000  # 30s margin
            test["timeout_ms"] = timeout_ms

    # Parse position changes
    move_actions = parse_setcoordinates(script, gen_msgs)
    write_actions = parse_write_calls(script, gen_msgs)
    remove_add_actions = parse_remove_add_actions(script, gen_msgs)

    actions = move_actions + write_actions + remove_add_actions
    # Sort by time so the simulation executes them in order
    actions.sort(key=lambda a: a.get("at_ms", 0))
    if actions:
        test["actions"] = actions

    if unsupported:
        test["unsupported_features"] = unsupported
        warnings.append(f"Unsupported features: {', '.join(unsupported)}")

    # Warn if script is complex and we couldn't extract steps
    if not steps and not timeout_is_success and not fail_on:
        warnings.append("Could not extract test steps from script — "
                        "manual conversion may be needed")

    return test, warnings


def list_firmware_files(root, csc_dir):
    """List all firmware files needed by a .csc file."""
    sim = root.find("simulation")
    if sim is None:
        return []

    firmware = set()
    for mt in sim.findall("motetype"):
        source_elem = mt.find("source")
        if source_elem is not None and source_elem.text:
            source = source_elem.text.strip()
            source = source.replace("[CONFIG_DIR]", csc_dir)
            fw_name = source_to_firmware_name(source)
            if fw_name:
                firmware.add(fw_name)

    return sorted(firmware)


def convert_csc(csc_path, contiki_dir=None, firmware_dir=None, js_native=False):
    """Convert a .csc file to csim JSON config dict.
    Returns (config_dict, warnings_list)."""
    csc_dir = os.path.dirname(os.path.abspath(csc_path))
    root = parse_csc(csc_path)
    sim = root.find("simulation")
    if sim is None:
        raise ValueError("No <simulation> element found in .csc file")

    warnings = []
    config = {}

    # Title
    title_elem = sim.find("title")
    if title_elem is not None and title_elem.text:
        # Use the test directory + filename as title
        rel_path = os.path.basename(csc_path)
        parent = os.path.basename(os.path.dirname(csc_path))
        config["title"] = f"{parent}/{rel_path.replace('.csc', '')}"

    # Seed
    seed = extract_seed(sim)
    if seed is not None:
        config["seed"] = seed

    # Startup delay
    delay_ms = extract_startup_delay(sim)
    if delay_ms > 0:
        config["startup_delay_ms"] = delay_ms

    # Radio medium
    rm = extract_radio_medium(sim)
    if rm:
        config["radiomedium"] = rm

    # Nodes
    nodes = extract_nodes(sim, csc_dir, firmware_dir)
    if nodes:
        config["nodes"] = nodes

    # Test script
    script = extract_script(root, csc_dir)
    if script:
        if js_native:
            # Embed raw JS for native execution via QuickJS
            cleaned = clean_script(script)
            timeout_ms, _, _, _ = parse_timeout(cleaned)
            if timeout_ms:
                config["timeout_ms"] = timeout_ms
            config["test"] = {"js_script_inline": cleaned}
        else:
            test_config, script_warnings = translate_script(script, nodes)
            warnings.extend(script_warnings)

            if test_config:
                # Timeout from script overrides any default
                if "timeout_ms" in test_config:
                    config["timeout_ms"] = test_config.pop("timeout_ms")

                config["test"] = test_config
    else:
        warnings.append("No test script found in .csc file")

    return config, warnings


def main():
    parser = argparse.ArgumentParser(
        description="Convert Cooja .csc files to csim JSON config"
    )
    parser.add_argument("csc_file", help="Path to .csc file")
    parser.add_argument("--contiki", help="Path to contiki-ng root")
    parser.add_argument("--firmware-dir", help="Directory containing .cc2538dk firmware files")
    parser.add_argument("-o", "--output", help="Output JSON file (default: stdout)")
    parser.add_argument("--list-firmware", action="store_true",
                        help="Just list needed firmware files and exit")
    parser.add_argument("--warn", action="store_true",
                        help="Show warnings on stderr")
    parser.add_argument("--js-native", action="store_true",
                        help="Embed raw JS script for native QuickJS execution "
                             "instead of converting to JSON steps")

    args = parser.parse_args()

    csc_path = args.csc_file
    if not os.path.exists(csc_path):
        print(f"Error: file not found: {csc_path}", file=sys.stderr)
        sys.exit(1)

    csc_dir = os.path.dirname(os.path.abspath(csc_path))

    if args.list_firmware:
        root = parse_csc(csc_path)
        firmware = list_firmware_files(root, csc_dir)
        for fw in firmware:
            print(fw)
        return

    config, warnings = convert_csc(csc_path, args.contiki, args.firmware_dir,
                                    js_native=args.js_native)

    if warnings and args.warn:
        for w in warnings:
            print(f"WARNING: {w}", file=sys.stderr)

    json_str = json.dumps(config, indent=4)

    if args.output:
        with open(args.output, "w") as f:
            f.write(json_str + "\n")
        print(f"Wrote {args.output}", file=sys.stderr)
    else:
        print(json_str)


if __name__ == "__main__":
    main()
