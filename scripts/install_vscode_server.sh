#!/usr/bin/env bash
# Install/repair VS Code Remote SSH server on this NUC (needs internet once).
set -euo pipefail
COMMIT="${1:-}"
if [[ -z "$COMMIT" ]]; then
  COMMIT=$(curl -fsSL https://update.code.visualstudio.com/api/commits/stable/linux-x64 | python3 -c 'import sys,json; print(json.load(sys.stdin)[0])')
fi
echo "Installing VS Code server commit: $COMMIT"
BASE="$HOME/.vscode-server"
mkdir -p "$BASE/bin/$COMMIT" "$BASE/cli/servers/Stable-$COMMIT"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

curl -fL --connect-timeout 20 --max-time 600 \
  -o "$TMP/server.tar.gz" \
  "https://vscode.download.prss.microsoft.com/dbazure/download/stable/${COMMIT}/vscode-server-linux-x64.tar.gz"
tar -xzf "$TMP/server.tar.gz" -C "$BASE/bin/$COMMIT" --strip-components=1
touch "$BASE/bin/$COMMIT/0"
ln -sfn "$BASE/bin/$COMMIT" "$BASE/cli/servers/Stable-$COMMIT/server"

curl -fL --connect-timeout 20 --max-time 180 \
  -o "$BASE/vscode-cli-${COMMIT}.tar.gz" \
  "https://vscode.download.prss.microsoft.com/dbazure/download/stable/${COMMIT}/vscode_cli_linux_x64_cli.tar.gz"
tar -xzf "$BASE/vscode-cli-${COMMIT}.tar.gz" -C "$BASE"
chmod +x "$BASE/code" 2>/dev/null || true
ls -lh "$BASE/vscode-cli-${COMMIT}.tar.gz"
echo "Done. Reconnect Remote-SSH."
