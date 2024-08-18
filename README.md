Projucer Exporter Settings
- Add "../../Libraries/libsofa/lib" and "../../Libraries/libsofa/dependencies/lib/win" to Extra Library Search Paths
- Set Runtime Library setting to "Use static runtime"

Todo
- Fix artifacts when changing azimuth/elevation (try convolving in frequency domain using the overlap and save method, rather than time domain convolution)
- Implement inter-aural time differences (currently only interaural level differences)
- Move HRTF prep work off-thread (when loading HRTF file)
- Add loop functionality!
