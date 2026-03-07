#!/usr/bin/env python3
"""
hdtSMP64 Automated Benchmark Pipeline

Runs benchmarks, captures Tracy profiles, exports CSV, and ingests into MCP Qdrant
for semantic performance analysis.

Usage:
    python benchmark_pipeline.py run --name test1 --frames 2000
    python benchmark_pipeline.py compare baseline test1

Configuration:
    Create .env file in benchmark/ directory with:
        MO2_PATH=C:\Path\To\ModOrganizer.exe
        SKYRIM_PATH=C:\Path\To\SkyrimSE.exe
"""

import subprocess
import json
import time
import os
import sys
import argparse
from pathlib import Path
from typing import Optional
import xml.etree.ElementTree as ET

try:
    from pydantic import Field
    from pydantic_settings import BaseSettings, SettingsConfigDict
except ImportError:
    print("[ERROR] pydantic-settings not installed")
    print("[ERROR] Run: cd benchmark && uv sync")
    sys.exit(1)

try:
    import winreg
except ImportError:
    winreg = None


class BenchmarkSettings(BaseSettings):
    """Configuration for benchmark pipeline"""

    model_config = SettingsConfigDict(
        env_file='.env',  # Load from benchmark/.env
        env_file_encoding='utf-8',
        extra='ignore'
    )

    # MO2 Configuration (preferred for modded setups)
    mo2_path: Optional[str] = Field(
        default=None,
        description="Path to ModOrganizer.exe (enables MO2 launching)"
    )
    mo2_executable: str = Field(
        default="SKSE",
        description="MO2 executable name (as configured in MO2, e.g., 'SKSE')"
    )
    mo2_profile: Optional[str] = Field(
        default=None,
        description="MO2 profile name (uses active profile if not specified)"
    )

    # Direct Skyrim launch (fallback if MO2 not configured)
    skyrim_path: Optional[str] = Field(
        default=None,
        description="Path to SkyrimSE.exe (auto-detects if not specified)"
    )

class BenchmarkPipeline:
    def __init__(self, project_root=None, settings=None):
        # If no project_root provided, determine it from script location
        # Script is in benchmark/, so project root is parent directory
        if project_root is None:
            script_dir = Path(__file__).parent
            project_root = script_dir.parent

        self.root = Path(project_root)
        self.settings = settings or BenchmarkSettings()
        self.results = self.root / "results"
        self.results.mkdir(exist_ok=True)
        self.config_path = self.root / "configs" / "configs.xml"
        self.log_path = Path.home() / "Documents" / "My Games" / "Skyrim Special Edition" / "SKSE" / "hdtSMP64.log"

        # Tracy tool paths (downloaded via 'just setup-tracy')
        tracy_tools = self.root / "tools" / "tracy"
        self.tracy_capture = tracy_tools / "tracy-capture.exe"
        self.tracy_csvexport = tracy_tools / "tracy-csvexport.exe"

    def find_skyrim_path(self):
        """
        Auto-detect Skyrim SE installation path via Steam registry.

        Returns:
            Path to SkyrimSE.exe if found, None otherwise
        """
        if not winreg:
            return None

        try:
            # Try Steam registry key for Skyrim SE (AppID 489830)
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                                r"SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\Steam App 489830")
            install_location, _ = winreg.QueryValueEx(key, "InstallLocation")
            winreg.CloseKey(key)

            skyrim_exe = Path(install_location) / "SkyrimSE.exe"
            if skyrim_exe.exists():
                return skyrim_exe
        except (FileNotFoundError, OSError):
            pass

        # Common installation paths as fallback
        common_paths = [
            Path(r"C:\Program Files (x86)\Steam\steamapps\common\Skyrim Special Edition\SkyrimSE.exe"),
            Path(r"C:\Steam\steamapps\common\Skyrim Special Edition\SkyrimSE.exe"),
            Path(r"D:\Steam\steamapps\common\Skyrim Special Edition\SkyrimSE.exe"),
            Path(r"E:\Steam\steamapps\common\Skyrim Special Edition\SkyrimSE.exe"),
        ]

        for path in common_paths:
            if path.exists():
                return path

        return None

    def update_config(self, enabled=False, frames=2000):
        """Update benchmark section in configs.xml"""
        try:
            tree = ET.parse(self.config_path)
            root = tree.getroot()

            # Find or create benchmark element
            benchmark = root.find('.//benchmark')
            if benchmark is None:
                benchmark = ET.SubElement(root, 'benchmark')

            # Update attributes
            benchmark.set('enabled', 'true' if enabled else 'false')
            benchmark.set('save', '')  # User loads manually
            benchmark.set('frames', str(frames))
            benchmark.set('exit-when-done', 'true')
            benchmark.set('suppress-ui', 'true')
            benchmark.set('quiet-mode', 'false')

            tree.write(self.config_path, encoding='utf-8', xml_declaration=True)
            print(f"[CONFIG] Updated: enabled={enabled}, frames={frames}")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to update config: {e}")
            return False

    def wait_for_completion(self, timeout_seconds=600):
        """
        Tail log file and wait for [BENCHMARK] Complete marker

        Args:
            timeout_seconds: Maximum wait time (default: 600s = 10 minutes)

        Returns:
            True if benchmark completed, False if timeout/error
        """
        print(f"[WAIT] Monitoring log: {self.log_path}")
        print(f"[WAIT] Timeout: {timeout_seconds}s")

        start_time = time.time()

        # Wait for log file to exist
        while not self.log_path.exists():
            if time.time() - start_time > 30:
                print(f"[ERROR] Log file not created after 30s: {self.log_path}")
                return False
            time.sleep(0.5)

        with open(self.log_path, 'r', encoding='utf-8', errors='replace') as f:
            # Seek to end
            f.seek(0, 2)

            while True:
                # Check timeout
                elapsed = time.time() - start_time
                if elapsed > timeout_seconds:
                    print(f"[ERROR] Timeout after {elapsed:.0f}s waiting for benchmark")
                    print(f"[ERROR] Skyrim may have crashed or benchmark not started")
                    return False

                line = f.readline()
                if not line:
                    time.sleep(0.1)
                    continue

                # Print benchmark-related lines
                if "[BENCHMARK]" in line:
                    print(f"  {line.strip()}")

                if "[BENCHMARK] Complete:" in line:
                    print("[WAIT] Benchmark completed!")
                    return True

                if "[BENCHMARK] Exiting" in line:
                    print("[WAIT] Application exiting...")
                    time.sleep(3)  # Wait for clean shutdown
                    return True

    def run_benchmark(self, name, frames=2000, manual_load=True, skyrim_exe=None):
        """
        Run complete benchmark workflow

        Args:
            name: Trace name (e.g., "highway_enabled_12entities")
            frames: Number of frames to capture
            manual_load: If True, prompts user to load save manually
            skyrim_exe: Path to SkyrimSE.exe (auto-detects if not provided)
        """
        # Security: Sanitize name to prevent path traversal
        safe_name = Path(name).name  # Strips directory components like "../"
        if not safe_name or safe_name in ('.', '..') or '/' in name or '\\' in name:
            print(f"[ERROR] Invalid benchmark name: '{name}'")
            print(f"[ERROR] Name must be a simple filename without path separators")
            return None

        if safe_name != name:
            print(f"[WARN] Sanitized benchmark name: '{name}' -> '{safe_name}'")
            name = safe_name

        print(f"\n{'='*60}")
        print(f"BENCHMARK: {name}")
        print(f"{'='*60}\n")

        # Validate frames
        if frames < 10 or frames > 10000:
            frames = max(10, min(frames, 10000))
            print(f"[WARN] Frames clamped to valid range [10, 10000]: {frames}")

        # 1. Validate Tracy tools exist
        if not self.tracy_capture.exists():
            print(f"[ERROR] Tracy capture tool not found: {self.tracy_capture}")
            print("[ERROR] Run 'just setup-tracy' to build Tracy tools")
            return None
        if not self.tracy_csvexport.exists():
            print(f"[ERROR] Tracy csvexport tool not found: {self.tracy_csvexport}")
            print("[ERROR] Run 'just setup-tracy' to build Tracy tools")
            return None

        # 2. Update config
        if not self.update_config(enabled=True, frames=frames):
            return None

        # 3. Start Tracy capture
        trace_dir = self.results / name
        trace_dir.mkdir(exist_ok=True)
        tracy_file = trace_dir / f"{name}.tracy"

        print(f"[TRACY] Starting capture...")
        tracy_proc = subprocess.Popen([
            str(self.tracy_capture),
            "-o", str(tracy_file),
            "-a", "127.0.0.1"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        time.sleep(2)  # Let Tracy initialize

        # 4. Launch Skyrim (via MO2 if configured, otherwise direct)
        if self.settings.mo2_path:
            # Launch via Mod Organizer 2
            mo2_path = Path(self.settings.mo2_path)
            if not mo2_path.exists():
                print(f"[ERROR] Mod Organizer 2 not found at: {mo2_path}")
                print("[ERROR] Update MO2_PATH in .env or remove it to use direct launch")
                tracy_proc.terminate()
                return None

            # Build MO2 command
            mo2_cmd = [str(mo2_path)]
            if self.settings.mo2_profile:
                mo2_cmd.extend(["-p", self.settings.mo2_profile])
                print(f"[MO2] Launching via profile: {self.settings.mo2_profile}")
            else:
                print(f"[MO2] Launching via active profile")

            mo2_cmd.append(self.settings.mo2_executable)
            print(f"[MO2] Executable: {self.settings.mo2_executable}")

            skyrim = subprocess.Popen(mo2_cmd)

        else:
            # Direct launch (no MO2)
            if skyrim_exe:
                skyrim_path = Path(skyrim_exe)
            elif self.settings.skyrim_path:
                skyrim_path = Path(self.settings.skyrim_path)
                print(f"[CONFIG] Using Skyrim path from .env: {skyrim_path}")
            else:
                skyrim_path = self.find_skyrim_path()
                if skyrim_path:
                    print(f"[CONFIG] Auto-detected Skyrim at: {skyrim_path}")

            if not skyrim_path or not skyrim_path.exists():
                print(f"[ERROR] Skyrim not found")
                print("[ERROR] Solutions:")
                print("[ERROR]   1. Create .env file with: SKYRIM_PATH=C:\\Path\\To\\SkyrimSE.exe")
                print("[ERROR]   2. Or pass --skyrim-exe to command line")
                print("[ERROR]   3. Or configure MO2_PATH for Mod Organizer 2 launching")
                tracy_proc.terminate()
                return None

            print(f"[SKYRIM] Launching {skyrim_path.name}...")
            skyrim = subprocess.Popen([str(skyrim_path)])

        if manual_load:
            print(f"\n{'='*60}")
            print(f"ACTION REQUIRED:")
            print(f"  1. Wait for Skyrim to load")
            print(f"  2. Load your benchmark save")
            print(f"  3. Stand still and let physics run")
            print(f"  4. Script will detect completion automatically")
            print(f"{'='*60}\n")

        # 4. Wait for benchmark completion
        completed = self.wait_for_completion()

        if not completed:
            print("[ERROR] Benchmark did not complete normally")
            tracy_proc.terminate()
            return None

        # 5. Stop Tracy capture
        time.sleep(2)  # Ensure final data is captured
        tracy_proc.terminate()
        tracy_proc.wait(timeout=10)
        print(f"[TRACY] Capture stopped")

        # 6. Export CSV
        csv_file = trace_dir / "trace.csv"
        print(f"[EXPORT] Creating CSV...")

        try:
            subprocess.run([
                str(self.tracy_csvexport), "-u",
                str(tracy_file)
            ], stdout=open(csv_file, 'w', encoding='utf-8'), check=True, timeout=60)
            print(f"[EXPORT] Saved to: {csv_file}")
        except Exception as e:
            print(f"[ERROR] CSV export failed: {e}")
            return None

        # 7. Ingest to MCP (if server is configured)
        try:
            self.ingest_to_mcp(name, csv_file)
        except Exception as e:
            print(f"[WARN] MCP ingestion skipped: {e}")

        # 8. Cleanup
        self.update_config(enabled=False)
        if tracy_file.exists():
            tracy_file.unlink()  # Delete .tracy, keep CSV
            print(f"[CLEANUP] Removed Tracy file (CSV preserved)")

        print(f"\n[SUCCESS] Benchmark complete: {name}")
        print(f"  Results: {csv_file}")

        return csv_file

    def ingest_to_mcp(self, trace_name, csv_file):
        """Ingest Tracy CSV into MCP Qdrant via hdt-log server"""
        mcp_server = self.root / "mcp-log-inspector" / "dist" / "index.js"

        if not mcp_server.exists():
            raise FileNotFoundError(f"MCP server not found: {mcp_server}")

        request = {
            "jsonrpc": "2.0",
            "id": 1,
            "method": "tools/call",
            "params": {
                "name": "ingest_tracy",
                "arguments": {
                    "csv_path": str(csv_file),
                    "trace_name": trace_name
                }
            }
        }

        proc = subprocess.Popen(
            ["node", str(mcp_server)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        response, err = proc.communicate(json.dumps(request))

        if proc.returncode != 0:
            raise Exception(f"MCP call failed: {err}")

        result = json.loads(response)
        if "error" in result:
            raise Exception(f"MCP error: {result['error']}")

        print(f"[MCP] Ingested: {result.get('result', 'OK')}")


def main():
    parser = argparse.ArgumentParser(description="hdtSMP64 Benchmark Pipeline")
    subparsers = parser.add_subparsers(dest='command', help='Command to run')

    # run command
    run_parser = subparsers.add_parser('run', help='Run benchmark')
    run_parser.add_argument('--name', required=True, help='Trace name')
    run_parser.add_argument('--frames', type=int, default=2000, help='Frame count (default: 2000)')
    run_parser.add_argument('--skyrim-exe', type=str, help='Path to SkyrimSE.exe (overrides .env and auto-detection)')

    # compare command (placeholder - use query_performance.py)
    compare_parser = subparsers.add_parser('compare', help='Compare two benchmarks')
    compare_parser.add_argument('baseline', help='Baseline trace name')
    compare_parser.add_argument('test', help='Test trace name')

    args = parser.parse_args()

    pipeline = BenchmarkPipeline()

    if args.command == 'run':
        skyrim_exe = getattr(args, 'skyrim_exe', None)
        pipeline.run_benchmark(args.name, args.frames, skyrim_exe=skyrim_exe)

    elif args.command == 'compare':
        print(f"Use: python query_performance.py compare {args.baseline} {args.test}")

    else:
        parser.print_help()


if __name__ == "__main__":
    main()
