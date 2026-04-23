from pathlib import Path
import os

Import("env")

REQUIRED_KEYS = (
    "WIFI_SSID",
    "WIFI_PASSWORD",
    "WS_HOST",
    "DEVICE_UUID",
)


def parse_dotenv(dotenv_path: Path) -> dict:
    values = {}
    if not dotenv_path.exists():
        return values

    for raw_line in dotenv_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            continue

        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()

        if not key:
            continue

        if (value.startswith('"') and value.endswith('"')) or (
            value.startswith("'") and value.endswith("'")
        ):
            value = value[1:-1]

        values[key] = value

    return values


def escape_for_define(value: str) -> str:
    return value.replace("\\", "\\\\").replace('"', '\\"')


project_dir = Path(env.subst("$PROJECT_DIR"))
dotenv_path = project_dir / ".env"
dotenv_values = parse_dotenv(dotenv_path)

resolved = {}
for key in REQUIRED_KEYS:
    # OS env overrides .env to support CI/CD secret injection
    resolved[key] = os.getenv(key, dotenv_values.get(key, ""))

missing = [k for k, v in resolved.items() if not v]
if missing:
    missing_str = ", ".join(missing)
    raise RuntimeError(
        f"Missing required env keys: {missing_str}. "
        "Create .env from .env.example or export variables in your shell."
    )

env.Append(
    CPPDEFINES=[
        ("ENV_WIFI_SSID", f'\\"{escape_for_define(resolved["WIFI_SSID"])}\\"'),
        ("ENV_WIFI_PASSWORD", f'\\"{escape_for_define(resolved["WIFI_PASSWORD"])}\\"'),
        ("ENV_WS_HOST", f'\\"{escape_for_define(resolved["WS_HOST"])}\\"'),
        ("ENV_DEVICE_UUID", f'\\"{escape_for_define(resolved["DEVICE_UUID"])}\\"'),
    ]
)

print("[dotenv] Loaded WIFI_SSID, WIFI_PASSWORD, WS_HOST, DEVICE_UUID")
