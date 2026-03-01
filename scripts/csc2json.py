#!/usr/bin/env python3
"""
Convert Cooja .csc simulation config (XML) to JSON format for csim.

Usage:
    python3 csc2json.py <input.csc> [-o output.json] [--firmware-dir DIR]

Supports both old format (motes as children of <simulation>) and new format
(motes nested inside <motetype>).
"""
import xml.etree.ElementTree as ET
import json
import sys
import os
import argparse


# Map Cooja mote type classes to firmware extensions
MOTE_TYPE_EXTENSIONS = {
    "ContikiMoteType": ".cooja",
    "SkyMoteType": ".sky",
    "CC2538MoteType": ".cc2538dk",
}


def get_extension_for_class(class_text):
    """Map a Cooja mote type class name to a firmware extension."""
    for key, ext in MOTE_TYPE_EXTENSIONS.items():
        if key in class_text:
            return ext
    return ".cooja"  # default


def extract_firmware_name(motetype_elem, ext):
    """Extract firmware path from a motetype element."""
    # Try <firmware> element first (Sky format)
    firmware_el = motetype_elem.find("firmware")
    if firmware_el is not None and firmware_el.text:
        return firmware_el.text.strip()

    # Try <commands> element: extract target name
    commands_el = motetype_elem.find("commands")
    if commands_el is not None and commands_el.text:
        # e.g. "$(MAKE) -j$(CPUS) udp-server.cooja TARGET=cooja"
        parts = commands_el.text.split()
        for part in parts:
            if part.endswith(ext):
                return part
            # Also check without extension
            for known_ext in MOTE_TYPE_EXTENSIONS.values():
                if part.endswith(known_ext):
                    return part

    # Try <source> element as fallback
    source_el = motetype_elem.find("source")
    if source_el is not None and source_el.text:
        base = os.path.splitext(os.path.basename(source_el.text.strip()))[0]
        return base + ext

    return None


def get_mote_id(mote_elem):
    """Extract mote ID from interface_config elements."""
    for ifc in mote_elem.findall("interface_config"):
        # The text content of interface_config is the class name
        text = ifc.text.strip() if ifc.text else ""
        if "MoteID" in text:
            id_el = ifc.find("id")
            if id_el is not None and id_el.text:
                return int(id_el.text.strip())
    return None


def convert_csc(csc_path, firmware_dir=None):
    """Parse a .csc file and return a JSON-compatible dict."""
    tree = ET.parse(csc_path)
    root = tree.getroot()
    sim = root.find("simulation")
    if sim is None:
        print(f"Error: no <simulation> element in {csc_path}", file=sys.stderr)
        sys.exit(1)

    result = {}

    # Title
    title_el = sim.find("title")
    if title_el is not None and title_el.text:
        result["title"] = title_el.text.strip()

    # Timeout from <events><logoutput>
    events_el = sim.find("events")
    if events_el is not None:
        logoutput = events_el.find("logoutput")
        if logoutput is not None and logoutput.text:
            result["timeout_ms"] = int(logoutput.text.strip())

    # Random seed
    seed_el = sim.find("randomseed")
    if seed_el is not None and seed_el.text:
        seed_text = seed_el.text.strip()
        if seed_text != "generated":
            try:
                result["seed"] = int(seed_text)
            except ValueError:
                pass

    # Build motetype lookup: identifier -> (class_text, firmware_name)
    motetype_map = {}
    motetypes = sim.findall("motetype")

    nodes = []

    for mt in motetypes:
        # The class is the text directly inside <motetype> (before child elements)
        class_text = mt.text.strip() if mt.text else ""
        ext = get_extension_for_class(class_text)
        fw_name = extract_firmware_name(mt, ext)

        # Get identifier for old-format lookup
        id_el = mt.find("identifier")
        if id_el is not None and id_el.text:
            identifier = id_el.text.strip()
            motetype_map[identifier] = (class_text, fw_name)

        # New format: motes nested inside motetype
        for mote in mt.findall("mote"):
            mote_id = get_mote_id(mote)
            firmware = fw_name
            if firmware_dir and firmware:
                # Rebase firmware path to firmware_dir
                firmware = os.path.join(firmware_dir, os.path.basename(firmware))
            node = {"firmware": firmware}
            if mote_id is not None:
                node["id"] = mote_id
            nodes.append(node)

    # Old format: motes directly under <simulation>
    for mote in sim.findall("mote"):
        mt_id_el = mote.find("motetype_identifier")
        if mt_id_el is not None and mt_id_el.text:
            mt_id = mt_id_el.text.strip()
            if mt_id in motetype_map:
                _, fw_name = motetype_map[mt_id]
                firmware = fw_name
                if firmware_dir and firmware:
                    firmware = os.path.join(firmware_dir, os.path.basename(firmware))
                mote_id = get_mote_id(mote)
                node = {"firmware": firmware}
                if mote_id is not None:
                    node["id"] = mote_id
                nodes.append(node)

    result["nodes"] = nodes
    return result


def main():
    parser = argparse.ArgumentParser(
        description="Convert Cooja .csc simulation config to JSON")
    parser.add_argument("csc_file", help="Input .csc file")
    parser.add_argument("-o", "--output", help="Output JSON file (default: stdout)")
    parser.add_argument("--firmware-dir",
                        help="Rebase firmware paths to this directory")
    args = parser.parse_args()

    result = convert_csc(args.csc_file, args.firmware_dir)
    json_str = json.dumps(result, indent=2) + "\n"

    if args.output:
        with open(args.output, "w") as f:
            f.write(json_str)
        print(f"Wrote {args.output}", file=sys.stderr)
    else:
        print(json_str, end="")


if __name__ == "__main__":
    main()
