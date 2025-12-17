# Copyright 2025 Analog Devices, Inc.
# All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of Analog Devices, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
# IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ------------------------------------------------------------------------------
import datetime
import os
import sys

sys.path.insert(0, os.path.abspath('.'))
# ------------------------------------------------------------------------------

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
#
# Values below are populated from package.xml by rosdoc2 if not set here.

# When this repo docs are generated as a submodule, use the parent repository name
repository = os.getenv('DOC_REPOSITORY', "tmcl_ros2")
project = 'adi_tmcl'

current_year = datetime.datetime.now().year
author = 'Analog Devices, Inc.'

copyright = f'{current_year}, {author}'

language = 'en'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "adi_doctools",
    "myst_parser",
    "sphinx.ext.todo",
]

needs_extensions = {
    'adi_doctools': '0.3'
}

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

# The master toctree document.
master_doc = 'index'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store',
                    "**/.work",             # act temp files
                    "**/.venv", "**/venv"   # local virtualenv
                    ]

# -- Options for rosdoc2 -------------------------------------------------------

# These settings are specific to rosdoc2, and if Sphinx is run without rosdoc2
# they will be safely ignored.
# None are required by default, so the lines below show the default values,
# therefore you will need to uncomment the lines and change their value
# if you want change the behavior of rosdoc2.
rosdoc2_settings = {
    # This setting, if True, will ensure breathe is part of the 'extensions',
    # and will set all of the breathe configurations, if not set, and override
    # settings as needed if they are set by this configuration.
    # 'enable_breathe': True,

    # This setting, if True, will ensure exhale is part of the 'extensions',
    # and will set all of the exhale configurations, if not set, and override
    # settings as needed if they are set by this configuration.
    # 'enable_exhale': True,

    # This setting, if provided, allows option specification for breathe
    # directives through exhale. If not set, exhale defaults will be used.
    # If an empty dictionary is provided, breathe defaults will be used.
    # 'exhale_specs_mapping': {},

    # This setting, if True, will ensure autodoc is part of the 'extensions'.
    # 'enable_autodoc': True,

    # This setting, if True, will ensure intersphinx is part of the 'extensions'.
    # 'enable_intersphinx': True,

    # This setting, if True, will have the 'html_theme' overridden to provide
    # a consistent style across all of the ROS documentation.
    'override_theme': False,

    # This setting, if True, will automatically extend the intersphinx mapping
    # using inventory files found in the cross-reference directory.
    # If false, the `found_intersphinx_mappings` variable will be in the global
    # scope when run with rosdoc2, and could be conditionally used in your own
    # Sphinx conf.py file.
    'automatically_extend_intersphinx_mapping': True,

    # Support markdown
    'support_markdown': True,

    # Allow additional extensions. If true, at runtime rosdoc2 will check to see if
    # non-default extensions are installed, and if so allow them. If false, only
    # extensions loaded by default by Sphinx or rosdoc2 installs are allowed.
    'allow_other_extensions': True,
}

# -- External docs configuration -----------------------------------------------

interref_repos = ['doctools']

# -- Custom extensions configuration -------------------------------------------

hide_collapsible_content = True
validate_links = False

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'harmonic'
html_static_path = ['doc/images']
intersphinx_disabled_reftypes = ["*"]
