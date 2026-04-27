#!/usr/bin/env python3
"""
Convert Unity test runner output to JUnit XML.

Unity output format (one result per line):
  file:line:test_name:PASS
  file:line:test_name:FAIL: reason
  file:line:test_name:IGNORE: reason

Summary line (last non-empty line):
  N Tests M Failures K Ignored

Usage:
  ./unity_tests.elf | python3 unity_to_junit.py [output.xml]
"""

import re
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone

RESULT_RE = re.compile(
    r"^(?P<file>[^:]+):(?P<line>\d+):(?P<name>[^:]+):(?P<status>PASS|FAIL|IGNORE)"
    r"(?::(?P<message>.+))?$"
)

def parse(lines):
    cases = []
    for line in lines:
        line = line.rstrip()
        m = RESULT_RE.match(line)
        if m:
            cases.append(m.groupdict())
    return cases

def build_xml(cases, suite_name="unity"):
    failures = sum(1 for c in cases if c["status"] == "FAIL")
    skipped  = sum(1 for c in cases if c["status"] == "IGNORE")

    suite = ET.Element("testsuite", {
        "name":      suite_name,
        "tests":     str(len(cases)),
        "failures":  str(failures),
        "skipped":   str(skipped),
        "errors":    "0",
        "timestamp": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S"),
    })

    for c in cases:
        tc = ET.SubElement(suite, "testcase", {
            "name":      c["name"],
            "classname": c["file"].replace("/", ".").removesuffix(".c"),
            "line":      c["line"],
        })
        if c["status"] == "FAIL":
            fail = ET.SubElement(tc, "failure", {"message": c["message"] or ""})
            fail.text = c["message"] or ""
        elif c["status"] == "IGNORE":
            skip = ET.SubElement(tc, "skipped", {"message": c["message"] or ""})
            skip.text = c["message"] or ""

    return ET.ElementTree(suite)

def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "unity_results.xml"
    lines = sys.stdin.readlines()
    cases = parse(lines)

    if not cases:
        print("unity_to_junit.py: no test results found in input", file=sys.stderr)
        sys.exit(1)

    tree = build_xml(cases)
    ET.indent(tree, space="  ")
    tree.write(out_path, encoding="unicode", xml_declaration=True)

    failures = sum(1 for c in cases if c["status"] == "FAIL")
    print(f"unity_to_junit.py: {len(cases)} tests, {failures} failures → {out_path}")
    sys.exit(1 if failures else 0)

if __name__ == "__main__":
    main()
