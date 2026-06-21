#!/usr/bin/env python
import os
import sys
import subprocess

godot_cpp_path = os.path.join(os.getcwd(), "godot-cpp")

if not os.path.exists(godot_cpp_path):
    print("godot-cpp not found. Cloning...")
    subprocess.run([
        "git", "clone",
        "https://github.com/godotengine/godot-cpp",
        godot_cpp_path,
        "--branch", "godot-4.2-stable",
        "--recursive"
    ], check=True)

env = SConscript("godot-cpp/SConstruct")

env.Append(CPPPATH=["src/"])

sources = Glob("src/*.cpp")

if env["platform"] == "macos":
    library = env.SharedLibrary(
        "bin/libwafer_stage_sim.{}.{}.framework/libwafer_stage_sim.{}.{}".format(
            env["platform"], env["target"], env["platform"], env["target"]
        ),
        source=sources,
    )
else:
    library = env.SharedLibrary(
        "bin/libwafer_stage_sim{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

Default(library)
