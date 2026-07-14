Import("env")
import os

env_path = os.path.join(env.get("PROJECT_DIR"), ".env")
if not os.path.isfile(env_path):
    print("load_env.py: no .env file found at {}, skipping".format(env_path))
else:
    with open(env_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            env.Append(CPPDEFINES=[(key.strip(), env.StringifyMacro(value.strip()))])
