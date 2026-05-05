# Artifacts

This directory stores generated files that are useful to keep locally but should not be treated as source code.

- `python/outputs/`: archived Python simulation figures and benchmark JSON.
- `cpp/outputs/`: archived C++ simulation figures and benchmark JSON.
- `cpp/build/`: archived C++ executables from the previous build.
- `cpp/objects/`: archived standalone object files that were previously in the C++ source root.
- `cache/`: archived Python and pytest caches.

Fresh runs will create new `outputs/`, `cpp/build/`, or cache directories as needed.
