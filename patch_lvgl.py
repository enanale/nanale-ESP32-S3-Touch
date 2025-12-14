import os
import shutil
from SCons.Script import Import

Import("env")

def remove_incompatible_dirs(source, target, env):
    libdeps_dir = env.subst("$PROJECT_LIBDEPS_DIR")
    pioenv = env.subst("$PIOENV")
    # Verify path exists
    base_dir = os.path.join(libdeps_dir, pioenv, "lvgl", "src", "draw")
    
    if not os.path.exists(base_dir):
        return

    # Walk through the directory to find 'helium' or 'neon' folders
    for root, dirs, files in os.walk(base_dir):
        for d in dirs:
            if d == "helium" or d == "neon":
                full_path = os.path.join(root, d)
                print(f"Removing incompatible directory: {full_path}")
                shutil.rmtree(full_path)

# Hook into the build process
# Executing immediately to ensure it runs before any file scanning happens if possible
remove_incompatible_dirs(None, None, env)

# Also add as pre-action just in case
env.AddPreAction("buildprog", remove_incompatible_dirs)
