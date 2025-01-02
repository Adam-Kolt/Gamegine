# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "FRC Gamegine"
copyright = "2024, Adam Koltuniuk"
author = "Adam Koltuniuk"
release = "0.1.0"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.autosectionlabel",
    "sphinx.ext.viewcode",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "furo"
html_static_path = ["_static"]
html_theme_options = {
    "light_logo": "GamegineDark.png",
    "dark_logo": "GamegineLight.png",
    "sidebar_hide_name": True,
    "navigation_with_keys": True,
}

autodoc_typehints = "description"

# Don't show class signature with the class' name.
autodoc_class_signature = "separated"


import os
import sys

print("Sphinx is using this Python executable:", sys.executable)
sys.path.insert(0, os.path.abspath("../"))
