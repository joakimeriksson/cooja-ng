#!/usr/bin/env python3
"""
Pin the Cooja-suite firmware cache contract (tools/csc2json.py and
run-cooja-tests.sh --clean):

  - two tests that build a same-named source from different directories
    never share a cached build, and neither do different make arguments;
  - a name depends only on the source directory relative to the Contiki-NG
    root and the make arguments — not on where the checkout lives, the cwd,
    or whether [CONTIKI_DIR] was resolved;
  - a build's own cache entry wins; failing that, only *shipped* firmware
    under the previous scheme's name is used, with a warning, and a local
    build left under that name is ignored;
  - "shipped" means tracked by a git checkout of the tree itself; a tree that
    merely sits inside another repository counts as not a checkout, and there
    only the prebuilt targets' files (.sky/.z1) are shipped — never a .cooja,
    since firmware/cooja is gitignored and the release package has none;
  - the Contiki-NG root is found above a .csc when --contiki is not given, so
    a name does not depend on the flag either;
  - --clean removes local builds and keeps shipped ones, by the same
    definition of "shipped" (csc2json --local-firmware).

A reordered lookup or a changed hash key reintroduces silent cross-directory
firmware reuse, which the suite itself cannot see: the wrong firmware often
passes.  Needs only python3 and git.

Usage: python3 tools/check-firmware-names.py [-v]
"""

import contextlib
import importlib.util
import io
import os
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

TOOLS = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "csc2json", os.path.join(TOOLS, "csc2json.py"))
csc2json = importlib.util.module_from_spec(spec)
spec.loader.exec_module(csc2json)


def git(cwd, *args):
    subprocess.run(["git", "-C", cwd, *args], check=True,
                   capture_output=True)


def touch(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write("x\n")


CSC = """<?xml version="1.0" encoding="UTF-8"?>
<simconf version="2023090101">
  <simulation>
    <title>t</title>
    <randomseed>1</randomseed>
    <motedelay_us>1000000</motedelay_us>
    <radiomedium>
      org.contikios.cooja.radiomediums.UDGM
      <transmitting_range>50.0</transmitting_range>
      <interference_range>100.0</interference_range>
      <success_ratio_tx>1.0</success_ratio_tx>
      <success_ratio_rx>1.0</success_ratio_rx>
    </radiomedium>
    <motetype>
      org.contikios.cooja.contikimote.ContikiMoteType
      <description>n</description>
      <source>{source}</source>
      <commands>$(MAKE) -j$(CPUS) node.cooja TARGET=cooja</commands>
      <mote>
        <interface_config>
          org.contikios.cooja.interfaces.Position
          <pos x="0" y="0" />
        </interface_config>
        <interface_config>
          org.contikios.cooja.contikimote.interfaces.ContikiMoteID
          <id>1</id>
        </interface_config>
      </mote>
    </motetype>
  </simulation>
  <plugin>
    org.contikios.cooja.plugins.ScriptRunner
    <plugin_config>
      <script>TIMEOUT(1000); log.testOK();</script>
      <active>true</active>
    </plugin_config>
  </plugin>
</simconf>
"""


def simulation(*motetypes):
    """A <simulation> with one mote per (description, source, target)."""
    xml = "<simulation>"
    for i, (desc, source, target) in enumerate(motetypes, 1):
        xml += f"""
  <motetype>
    <description>{desc}</description>
    <source>{source}</source>
    <commands>$(MAKE) -j$(CPUS) node.{target} TARGET={target}</commands>
    <mote>
      <interface_config>org.contikios.cooja.interfaces.MoteID
        <id>{i}</id>
      </interface_config>
    </mote>
  </motetype>"""
    return ET.fromstring(xml + "\n</simulation>")


class FakeTree(unittest.TestCase):
    """A scratch Cooja-NG tree (CSIM_DIR) beside a scratch Contiki-NG tree."""

    def setUp(self):
        self.tmp = os.path.realpath(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp)
        self.contiki = os.path.join(self.tmp, "contiki-ng")
        self.csc_dir = os.path.join(self.contiki, "tests", "07-a")
        os.makedirs(self.csc_dir)
        self.use_tree(os.path.join(self.tmp, "csim"))

    def use_tree(self, csim):
        os.makedirs(csim, exist_ok=True)
        self.csim = csim
        self.fw = os.path.join(csim, "firmware", "z1")
        os.makedirs(self.fw, exist_ok=True)
        saved = csc2json.CSIM_DIR
        csc2json.CSIM_DIR = csim
        self.addCleanup(setattr, csc2json, "CSIM_DIR", saved)
        self.clear_caches()
        self.addCleanup(self.clear_caches)

    @staticmethod
    def clear_caches():
        csc2json._csim_is_checkout.cache_clear()
        csc2json._tracked_names.cache_clear()

    def nodes(self, *motetypes):
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            nodes = csc2json.extract_nodes(
                simulation(*motetypes), self.csc_dir,
                os.path.join(self.csim, "firmware", "cooja"), self.contiki)
        return [n["firmware"] for n in nodes], err.getvalue()


class Naming(unittest.TestCase):
    def test_same_name_different_directories(self):
        a = csc2json.firmware_variant("node", "/c/tests/14-rpl-lite/code/node.c", [], "/c")
        b = csc2json.firmware_variant("node", "/c/tests/15-rpl-classic/code/node.c", [], "/c")
        self.assertNotEqual(a, b)

    def test_make_args_distinguish(self):
        src = "/c/tests/07-a/code/node.c"
        self.assertNotEqual(
            csc2json.firmware_variant("node", src, [], "/c"),
            csc2json.firmware_variant("node", src, ["MAKE_ROUTING=MAKE_ROUTING_RPL_CLASSIC"], "/c"))

    def test_make_args_order_irrelevant(self):
        src = "/c/tests/07-a/code/node.c"
        self.assertEqual(
            csc2json.firmware_variant("node", src, ["A=1", "B=2"], "/c"),
            csc2json.firmware_variant("node", src, ["B=2", "A=1"], "/c"))

    def test_same_on_every_machine(self):
        rel = "tests/15-rpl-classic/code/node.c"
        names = {csc2json.firmware_variant("node", f"{root}/{rel}", [], root)
                 for root in ("/home/a/contiki-ng", "/srv/ci/contiki-ng")}
        names.add(csc2json.firmware_variant("node", "[CONTIKI_DIR]/" + rel, [], None))
        self.assertEqual(len(names), 1, names)

    def test_independent_of_cwd(self):
        src = "[CONTIKI_DIR]/tests/15-rpl-classic/code/node.c"
        cwd = os.getcwd()
        try:
            names = set()
            for d in ("/", tempfile.gettempdir()):
                os.chdir(d)
                names.add(csc2json.firmware_variant("node", src, [], None))
        finally:
            os.chdir(cwd)
        self.assertEqual(len(names), 1, names)

    def test_contiki_dir_placeholder_resolved(self):
        self.assertEqual(
            csc2json.resolve_source("[CONTIKI_DIR]/tests/x/node.c", "/cfg", "/c"),
            "/c/tests/x/node.c")
        self.assertEqual(
            csc2json.resolve_source("[CONFIG_DIR]/code/node.c", "/cfg", None),
            "/cfg/code/node.c")


class ContikiRoot(FakeTree):
    """--contiki may be omitted: the root is found above the .csc."""

    def setUp(self):
        super().setUp()
        self.csc = os.path.join(self.csc_dir, "t.csc")
        with open(self.csc, "w") as f:
            f.write(CSC.format(source="[CONFIG_DIR]/code/node.c"))
        self.env = dict(os.environ)
        self.env.pop("CONTIKI_DIR", None)
        self.addCleanup(lambda: (os.environ.clear(), os.environ.update(self.env)))
        os.environ.clear()
        os.environ.update(self.env)

    def mark_contiki_root(self):
        touch(os.path.join(self.contiki, "Makefile.include"))
        os.makedirs(os.path.join(self.contiki, "os"))

    def names(self, contiki):
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            config, _ = csc2json.convert_csc(
                self.csc, contiki, os.path.join(self.csim, "firmware", "cooja"),
                js_native=True)
        return sorted({os.path.basename(n["firmware"]) for n in config["nodes"]})

    def test_found_above_the_csc(self):
        self.mark_contiki_root()
        self.assertEqual(csc2json.find_contiki_dir(self.csc_dir), self.contiki)
        self.assertEqual(self.names(None), self.names(self.contiki))

    def test_from_environment(self):
        os.environ["CONTIKI_DIR"] = self.contiki
        self.assertEqual(csc2json.find_contiki_dir(self.csc_dir), self.contiki)
        self.assertEqual(self.names(None), self.names(self.contiki))

    def test_from_csim_conf(self):
        with open(os.path.join(self.csim, "csim.conf"), "w") as f:
            f.write(f"CONTIKI_DIR={self.contiki}\n")
        self.assertEqual(csc2json.find_contiki_dir(self.csc_dir), self.contiki)

    def test_none_is_an_error_with_firmware_dir(self):
        self.assertIsNone(csc2json.find_contiki_dir(self.csc_dir))
        with self.assertRaises(csc2json.ConversionError) as cm:
            self.names(None)
        self.assertIn("--contiki", str(cm.exception))


class Lookup(FakeTree):
    def setUp(self):
        super().setUp()
        git(self.csim, "init", "-q")
        self.src = "[CONFIG_DIR]/code/node.c"
        self.variant = csc2json.firmware_variant(
            "node", os.path.join(self.csc_dir, "code", "node.c"), [], self.contiki)

    def test_own_build_first(self):
        touch(os.path.join(self.fw, "node.z1"))
        git(self.csim, "add", "firmware/z1/node.z1")
        touch(os.path.join(self.fw, self.variant + ".z1"))
        fw, err = self.nodes(("n", self.src, "z1"))
        self.assertEqual(fw, [os.path.join(self.fw, self.variant + ".z1")])
        self.assertEqual(err, "")

    def test_tracked_legacy_used_with_warning(self):
        touch(os.path.join(self.fw, "node.z1"))
        git(self.csim, "add", "firmware/z1/node.z1")
        fw, err = self.nodes(("n", self.src, "z1"))
        self.assertEqual(fw, [os.path.join(self.fw, "node.z1")])
        self.assertIn("WARNING: ", err)
        self.assertIn("tests/07-a/code", err)

    def test_untracked_legacy_ignored(self):
        touch(os.path.join(self.fw, "node.z1"))
        fw, err = self.nodes(("n", self.src, "z1"))
        self.assertEqual(fw, [os.path.join(self.fw, self.variant + ".z1")])
        self.assertEqual(err, "")

    def test_tracked_elsewhere_is_not_tracked_here(self):
        touch(os.path.join(self.fw, "sub", "node.z1"))
        git(self.csim, "add", "firmware/z1/sub/node.z1")
        touch(os.path.join(self.fw, "node.z1"))
        self.assertFalse(csc2json.is_shipped_firmware(
            os.path.join(self.fw, "node.z1")))

    def test_same_name_different_directories(self):
        fw, _ = self.nodes(("a", "[CONTIKI_DIR]/tests/14-rpl-lite/code/node.c", "z1"),
                           ("b", "[CONTIKI_DIR]/tests/15-rpl-classic/code/node.c", "z1"))
        self.assertEqual(len(set(fw)), 2, fw)


class LookupOutsideCheckout(FakeTree):
    """The tree is not a git checkout (a release archive, or unpacked inside
    another repository): a prebuilt .z1 under the old name is shipped, but a
    plain-named .cooja is a stale local build and must not be reused."""

    def setUp(self):
        super().setUp()
        git(self.tmp, "init", "-q")
        self.cooja = os.path.join(self.csim, "firmware", "cooja")
        os.makedirs(self.cooja)
        self.src = "[CONFIG_DIR]/code/node.c"

    def test_prebuilt_legacy_used_with_warning(self):
        touch(os.path.join(self.fw, "node.z1"))
        fw, err = self.nodes(("n", self.src, "z1"))
        self.assertEqual(fw, [os.path.join(self.fw, "node.z1")])
        self.assertIn("WARNING: ", err)

    def test_plain_named_cooja_ignored(self):
        touch(os.path.join(self.cooja, "node.cooja"))
        variant = csc2json.firmware_variant(
            "node", os.path.join(self.csc_dir, "code", "node.c"), [], self.contiki)
        fw, err = self.nodes(("n", self.src, "cooja"))
        self.assertEqual(fw, [os.path.join(self.cooja, variant + ".cooja")])
        self.assertEqual(err, "")


class Shipped(FakeTree):
    """Which git layouts count as a checkout of the tree."""

    def firmware(self):
        path = os.path.join(self.fw, "node.z1")
        touch(path)
        return path

    def test_own_checkout(self):
        git(self.csim, "init", "-q")
        path = self.firmware()
        self.assertFalse(csc2json.is_shipped_firmware(path))
        git(self.csim, "add", "firmware/z1/node.z1")
        self.clear_caches()
        self.assertTrue(csc2json.is_shipped_firmware(path))

    def test_nested_in_another_repository(self):
        # The tree unpacked, untracked, inside some other repository: git
        # would call every file in it untracked.  It is not a checkout.
        git(self.tmp, "init", "-q")
        self.assertTrue(csc2json.is_shipped_firmware(self.firmware()))

    def test_not_a_repository(self):
        self.assertTrue(csc2json.is_shipped_firmware(self.firmware()))

    def test_cooja_is_never_shipped(self):
        # firmware/cooja is gitignored and the release package has no
        # firmware, so outside a checkout a .cooja can only be a local build.
        path = os.path.join(self.csim, "firmware", "cooja", "node.cooja")
        touch(path)
        self.assertFalse(csc2json.is_shipped_firmware(path))
        git(self.tmp, "init", "-q")
        self.clear_caches()
        self.assertFalse(csc2json.is_shipped_firmware(path))

    def test_outside_the_tree(self):
        path = os.path.join(self.tmp, "elsewhere", "node.z1")
        touch(path)
        self.assertFalse(csc2json.is_shipped_firmware(path))


class Clean(FakeTree):
    """run-cooja-tests.sh --clean, in a scratch copy of the tree.  It stops
    at the missing test_runner, after cleaning."""

    def setUp(self):
        super().setUp()
        os.makedirs(os.path.join(self.csim, "tools"))
        for tool in ("run-cooja-tests.sh", "csc2json.py"):
            shutil.copy(os.path.join(TOOLS, tool),
                        os.path.join(self.csim, "tools"))
        self.cooja = os.path.join(self.csim, "firmware", "cooja")
        self.shipped = os.path.join(self.fw, "node.z1")
        self.local = os.path.join(self.fw, "node-abcdef.z1")
        self.cooja_plain = os.path.join(self.cooja, "node.cooja")
        self.cooja_hashed = os.path.join(self.cooja, "node-abcdef.cooja")
        for f in (self.shipped, self.local, self.cooja_plain, self.cooja_hashed):
            touch(f)

    def clean(self):
        env = dict(os.environ, CONTIKI_DIR=self.contiki)
        env.pop("CSIM_DIR", None)
        r = subprocess.run(
            ["bash", os.path.join(self.csim, "tools", "run-cooja-tests.sh"), "--clean"],
            env=env, capture_output=True, text=True, check=False)
        out = r.stdout + r.stderr
        # The script must have got past the CLEAN block, not died before it.
        self.assertIn("CLEAN " + self.fw, out)
        self.assertIn("CLEAN " + self.cooja, out)
        return out

    def assertKept(self, *paths):
        for p in paths:
            self.assertTrue(os.path.exists(p), p)

    def assertRemoved(self, *paths):
        for p in paths:
            self.assertFalse(os.path.exists(p), p)

    def test_own_checkout(self):
        git(self.csim, "init", "-q")
        git(self.csim, "add", "firmware/z1/node.z1")
        out = self.clean()
        self.assertKept(self.shipped)
        self.assertRemoved(self.local, self.cooja_plain, self.cooja_hashed)
        self.assertNotIn("not a git checkout", out)

    def test_gitignored_directory(self):
        # firmware/cooja is gitignored in the real tree; ignored files are
        # still local builds and must be cleaned.
        git(self.csim, "init", "-q")
        with open(os.path.join(self.csim, ".gitignore"), "w") as f:
            f.write("firmware/cooja/\n")
        git(self.csim, "add", ".gitignore", "firmware/z1/node.z1")
        self.clean()
        self.assertKept(self.shipped)
        self.assertRemoved(self.local, self.cooja_plain, self.cooja_hashed)

    def test_nested_in_another_repository(self):
        git(self.tmp, "init", "-q")
        out = self.clean()
        self.assertIn("not a git checkout", out)
        self.assertKept(self.shipped, self.local)
        self.assertRemoved(self.cooja_plain, self.cooja_hashed)

    def test_not_a_repository(self):
        out = self.clean()
        self.assertIn("not a git checkout", out)
        self.assertKept(self.shipped, self.local)
        self.assertRemoved(self.cooja_plain, self.cooja_hashed)


if __name__ == "__main__":
    unittest.main()
