
zephyr_library()
zephyr_library_sources(m90e26.c)

# Add emulator source file if enabled
zephyr_library_sources_ifdef(CONFIG_M90E26_EMUL m90e26_emul.c)