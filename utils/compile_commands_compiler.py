import glob
import os
import json
out = []
for p in glob.glob("build/*/compile_commands.json"):
    with open(p) as f:
        out += json.load(f)
with open("build/compile_commands.json","w") as f:
    json.dump(out,f)
print("Wrote build/compile_commands.json with", len(out), "entries")
