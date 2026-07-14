#!/usr/bin/env python3
"""
Remove AVX build variant configurations from hdtSMP64.vcxproj.

Keeps: {VERSION}_CUDA, {VERSION}_NOCUDA, {VERSION}_CUDA_DEBUG, {VERSION}_NOCUDA_DEBUG
Removes: anything with _NoAVX, _AVX_, _AVX2_, _AVX512_ in the configuration name
Also sets global EnableEnhancedInstructionSet to NotSet for kept configs.
"""

import xml.etree.ElementTree as ET
import re
import sys
import shutil
from pathlib import Path

NS = "http://schemas.microsoft.com/developer/msbuild/2003"
ET.register_namespace("", NS)


def tag(name):
    return f"{{{NS}}}{name}"


def is_avx_config(config_name):
    """Return True if this config should be removed (has an AVX variant suffix)."""
    # Match _NoAVX, _AVX_, _AVX2_, _AVX512_ anywhere in name
    # Keeps plain _AVX configs (no suffix) as the unified base configuration
    return bool(re.search(r"_(NoAVX|AVX_|AVX2|AVX512)", config_name))


def get_config_name(condition):
    """Extract config name from Condition attribute like '$(Configuration)|$(Platform)'=='FOO|x64'"""
    m = re.search(r"'=='\s*'([^|]+)\|", condition)
    if not m:
        m = re.search(r'Include="([^|]+)\|', condition)
    return m.group(1) if m else None


def strip_avx(vcxproj_path):
    tree = ET.parse(vcxproj_path)
    root = tree.getroot()

    removed_configs = set()
    elements_to_remove = []

    # Pass 1: collect ProjectConfiguration entries to remove
    for ig in root.findall(f".//{tag('ItemGroup')}"):
        for pc in ig.findall(tag("ProjectConfiguration")):
            include = pc.get("Include", "")
            config_name = include.split("|")[0]
            if is_avx_config(config_name):
                elements_to_remove.append((ig, pc))
                removed_configs.add(config_name)

    # Pass 2: collect PropertyGroup and ItemDefinitionGroup with matching Condition
    for elem in root:
        cond = elem.get("Condition", "")
        config_name = get_config_name(cond)
        if config_name and config_name in removed_configs:
            elements_to_remove.append((root, elem))
        # Also check nested children
        for child in elem:
            child_cond = child.get("Condition", "")
            child_config = get_config_name(child_cond)
            if child_config and child_config in removed_configs:
                elements_to_remove.append((elem, child))

    # Remove collected elements
    for parent, child in elements_to_remove:
        try:
            parent.remove(child)
        except ValueError:
            pass  # already removed

    HIGHWAY_FILES = {"hdtHighwayAABB.cpp", "hdtHighwaySkinning.cpp"}

    # Pass 3: set EnableEnhancedInstructionSet to NotSet for non-Highway files.
    # Highway files must keep AdvancedVectorExtensions512 so the compiler accepts
    # AVX512 intrinsics during foreach_target multi-target compilation.
    for cl in root.findall(f".//{tag('ClCompile')}"):
        include = cl.get("Include", "")
        filename = include.replace("\\", "/").split("/")[-1]
        is_highway = filename in HIGHWAY_FILES
        for eis in cl.findall(f".//{tag('EnableEnhancedInstructionSet')}"):
            current = eis.text or ""
            if is_highway:
                # Ensure Highway files always have AVX512
                eis.text = "AdvancedVectorExtensions512"
            elif current in (
                "AdvancedVectorExtensions",
                "AdvancedVectorExtensions2",
                "AdvancedVectorExtensions512",
            ):
                eis.text = "NotSet"

    print(f"Removed {len(removed_configs)} configurations:")
    for c in sorted(removed_configs):
        print(f"  - {c}")

    # Backup original
    backup = Path(vcxproj_path).with_suffix(".vcxproj.bak")
    shutil.copy2(vcxproj_path, backup)
    print(f"Backup saved to {backup}")

    tree.write(vcxproj_path, encoding="utf-8", xml_declaration=True)
    print(f"Written: {vcxproj_path}")


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "hdtSMP64/hdtSMP64.vcxproj"
    strip_avx(path)
