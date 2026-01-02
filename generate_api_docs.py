import os
import pkgutil
import inspect
import importlib
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import List, Dict, Any, Optional

# Add project root to path so we can import gamegine
project_root = Path(__file__).parent.resolve()
sys.path.insert(0, str(project_root))

import gamegine

OUTPUT_DIR = project_root / "docs-site/content/docs/api"
IGNORE_MODULES = ["gamegine.tests", "gamegine.examples"]

@dataclass
class DocItem:
    name: str
    obj: Any
    doc: str
    kind: str  # "class", "function", "module"
    members: List["DocItem"] = None

def get_doc(obj: Any) -> str:
    """Get cleaned docstring."""
    doc = inspect.getdoc(obj)
    if not doc:
        return ""
    # Escape MDX special characters
    # We escape < and > to prevent them being interpreted as JSX tags
    return doc.replace("{", "\\{").replace("}", "\\}").replace("<", "&lt;").replace(">", "&gt;")

def is_gamegine_module(module) -> bool:
    if not hasattr(module, "__name__"):
        return False
    return module.__name__.startswith("gamegine") and not any(ign in module.__name__ for ign in IGNORE_MODULES)

def extract_module_docs(module_name: str, module) -> DocItem:
    """Extract documentation structure from a module."""
    members = []
    
    # Inspect members
    for name, obj in inspect.getmembers(module):
        if name.startswith("_"):
            continue
            
        # Classes
        if inspect.isclass(obj):
            if obj.__module__ == module_name: # Only define if defined in this module
                class_members = []
                # Methods
                for m_name, m_obj in inspect.getmembers(obj):
                     if not m_name.startswith("_") or m_name == "__init__":
                        if inspect.isfunction(m_obj) or inspect.ismethod(m_obj):
                             class_members.append(DocItem(m_name, m_obj, get_doc(m_obj), "method"))
                
                members.append(DocItem(name, obj, get_doc(obj), "class", class_members))

        # Functions
        elif inspect.isfunction(obj):
            if obj.__module__ == module_name:
                members.append(DocItem(name, obj, get_doc(obj), "function"))

    return DocItem(module_name, module, get_doc(module), "module", members)

def generate_mdx(item: DocItem, depth: int = 1) -> str:
    """Generate MDX content for a DocItem."""
    mdx = ""
    
    if item.kind == "module":
        mdx += f"---\ntitle: {item.name}\n---\n\n"
        mdx += f"# {item.name}\n\n"
        if item.doc:
            mdx += f"{item.doc}\n\n"
        
        # Organize members
        classes = [m for m in item.members if m.kind == "class"]
        functions = [m for m in item.members if m.kind == "function"]
        
        if classes:
            mdx += "## Classes\n\n"
            for cls in classes:
                mdx += generate_mdx(cls, depth + 1)
        
        if functions:
            mdx += "## Functions\n\n"
            for func in functions:
                mdx += generate_mdx(func, depth + 1)
                
    elif item.kind == "class":
        mdx += f"### {item.name}\n\n"
        mdx += f"```python\nclass {item.name}\n```\n\n"
        if item.doc:
            mdx += f"{item.doc}\n\n"
        
        if item.members:
            for member in item.members:
                mdx += generate_mdx(member, depth + 1)
                
    elif item.kind == "method":
        # Skip __init__ if no docstring
        if item.name == "__init__" and not item.doc:
            return ""
            
        name_display = item.name
        if item.name == "__init__":
             name_display = "__init__"
             
        # Signature
        try:
            sig = str(inspect.signature(item.obj))
        except:
            sig = "()"
            
        mdx += f"#### `{name_display}`\n\n"
        mdx += f"```python\ndef {item.name}{sig}\n```\n\n"
        if item.doc:
            mdx += f"{item.doc}\n\n"

    elif item.kind == "function":
        try:
            sig = str(inspect.signature(item.obj))
        except:
            sig = "()"
            
        mdx += f"### {item.name}\n\n"
        mdx += f"```python\ndef {item.name}{sig}\n```\n\n"
        if item.doc:
            mdx += f"{item.doc}\n\n"

    return mdx

def main():
    print(f"Generating API docs in {OUTPUT_DIR}")
    if not OUTPUT_DIR.exists():
        OUTPUT_DIR.mkdir(parents=True)
        
    # Generate meta.json for ordering
    meta_pages = []

    # Recursively find modules
    packages = ["gamegine"]
    
    # We will just traverse relevant subpackages manually for simplicity/control
    subpackages = [
        "gamegine.simulation",
        "gamegine.render", 
        "gamegine.analysis",
        "gamegine.representation",
        "gamegine.utils"
    ]
    
    all_modules = []
    
    # helper to walk
    def walk_package(pkg_name):
        try:
            mod = importlib.import_module(pkg_name)
            if hasattr(mod, "__path__"):
                for _, name, ispkg in pkgutil.walk_packages(mod.__path__, mod.__name__ + "."):
                    if any(ign in name for ign in IGNORE_MODULES):
                        continue
                    all_modules.append(name)
            else:
                all_modules.append(pkg_name)
        except Exception as e:
            print(f"Error importing {pkg_name}: {e}")

    for pkg in subpackages:
        walk_package(pkg)
        
    # Deduplicate
    all_modules = sorted(list(set(all_modules)))
    
    for mod_name in all_modules:
        try:
            module = importlib.import_module(mod_name)
            doc_item = extract_module_docs(mod_name, module)
            
            # Only write if it has members or doc
            if doc_item.members or doc_item.doc:
                filename = mod_name.replace("gamegine.", "") + ".mdx"
                # Flatten hierarchy for now: api/simulation.match.mdx
                
                content = generate_mdx(doc_item)
                
                out_path = OUTPUT_DIR / filename
                with open(out_path, "w") as f:
                    f.write(content)
                
                print(f"Generated {filename}")
                meta_pages.append(filename.replace(".mdx", ""))
        except Exception as e:
            print(f"Skipping {mod_name}: {e}")

    # Write meta.json
    import json
    with open(OUTPUT_DIR / "meta.json", "w") as f:
        json.dump({"pages": meta_pages}, f, indent=2)

if __name__ == "__main__":
    main()
