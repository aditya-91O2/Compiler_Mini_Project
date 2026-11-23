"""One-command demo: compile .rbt -> src/main.cpp, then build with PlatformIO if available."""
import os, shutil, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)                                   # always run from the project folder
src = sys.argv[1] if len(sys.argv) > 1 else os.path.join("examples", "test1.rbt")

r = subprocess.run([
    sys.executable, "compiler.py", src,
    "-o", os.path.join("src", "main.cpp"),
    "--target", "dc",
    "--ast", "--ir", "--trace"
]) 
if r.returncode != 0:
    sys.exit(r.returncode)                       # compile error already printed

pio = shutil.which("pio") or shutil.which("platformio")
if pio:
    subprocess.run([pio, "run"], check=True)
    print("\nBuild done. Now press F1 -> 'Wokwi: Start Simulator'")
else:
    print("\nGenerated src/main.cpp. 'pio' is not on PATH, so build from VS Code instead:")
    print("  PlatformIO sidebar -> Build (or Ctrl+Alt+B), then F1 -> 'Wokwi: Start Simulator'")
