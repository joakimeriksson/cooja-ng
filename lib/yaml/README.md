# libyaml 0.2.5 — parser half only

Vendored from https://github.com/yaml/libyaml (tag 0.2.5, MIT — see LICENSE),
unmodified.  Only the *parser* side is shipped: `api.c parser.c scanner.c
reader.c loader.c` + the two headers.  `emitter.c writer.c dumper.c` are
deliberately absent — Cooja-NG writes YAML with its own canonical writer
(`src/sim/sim_config_yaml.c`), which keeps comments and key order the way a
human would; libyaml's emitter drops comments.

Vendored rather than pkg-config-detected on purpose: a build-time-detected
dependency silently drops features in a release build (the GNU Lightning
lesson), and the static release tarballs must always be able to read YAML.

To update: copy the same seven files from the new release and bump the
version defines in the Makefile (`YAML_VERSION_*`).
