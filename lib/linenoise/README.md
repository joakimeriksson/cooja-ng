# linenoise — vendored line editor for the Cooja-NG shell

Vendored from https://github.com/antirez/linenoise (BSD-2-Clause — see
LICENSE), commit `a473823d74b93eab2ba83480df16ed37617493f2` (2026-05-02).
Two files: `linenoise.c`, `linenoise.h`.

Used by `src/services/shell_service.c` through the **multiplexing API**
(`linenoiseEditStart` / `linenoiseEditFeed` / `linenoiseEditStop` plus
`linenoiseHide` / `linenoiseShow`), so the single-threaded simulation loop
can feed keystrokes between event slices and redraw the prompt around
asynchronous node output.  The blocking `linenoise()` call is not used.

Vendored rather than pkg-config-detected on purpose (the libyaml rationale):
a build-time-detected dependency silently drops a feature in a release
build, and the shell must work in every binary.

## One local change — re-apply on update

`enableRawMode()` upstream clears `OPOST`.  Here that line is commented out
(search for "COOJA-NG LOCAL CHANGE") so output post-processing (`\n` →
`\r\n`) stays on while the prompt is live: the simulator prints node console
lines and command output with plain `\n` through stdio, and without ONLCR
every such line staircases across the terminal.  linenoise's own edit-line
output uses only `\r` and escape sequences, which OPOST does not touch.

## To update

Copy the same three files from the new upstream commit, bump the commit hash
above, and re-apply the `OPOST` change.  The build rule
(`$(LINENOISE_BUILD_DIR)/%.o` in the Makefile) compiles it with warnings
silenced like the other vendored code.
