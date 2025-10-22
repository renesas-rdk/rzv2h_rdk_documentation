# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'RZ/V2H Robotic Development Kit User Manual'
copyright = '2025, Renesas Electronics Corporation'
author = 'Renesas Electronics Corporation'
release = '0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['myst_parser', 'sphinxcontrib.spelling', 'sphinx_copybutton', 'sphinx.ext.autodoc']

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_favicon = '../renesas_favicon.png'

# -- Options for LaTeX output --------------------------------------------------
latex_engine = 'lualatex'

latex_elements = {
    'passoptionstopackages': r'''
\PassOptionsToPackage{svgnames}{xcolor}
''',
    'fontpkg': r'''
\setmainfont{TeX Gyre Termes}      % Times-like
\setsansfont{TeX Gyre Heros}       % Helvetica-like
\setmonofont[Scale=MatchLowercase]{TeX Gyre Cursor} % Courier-like
''',
    'preamble': r'''
\usepackage[titles]{tocloft}
\cftsetpnumwidth {1.25cm}\cftsetrmarg{1.5cm}
\setlength{\cftchapnumwidth}{0.75cm}
\setlength{\cftsecindent}{\cftchapnumwidth}
\setlength{\cftsecnumwidth}{1.25cm}
''',
    "sphinxsetup": r"""
        verbatimwrapslines=true,     % keep wrapping long lines
        verbatimcontinued=,          % hide the wrap marker
        verbatimvisiblespace=,       % also hide the little visible-space glyph before breaks
    """,
'fncychap': r'\usepackage[Bjornstrup]{fncychap}',
'printindex': r'\footnotesize\raggedright\printindex',
}

latex_documents = [
    ('index',
     'WS125_Robotic_Development_Kit_User_Manual.tex',
     r'RZ/V2H Robotic Development Kit\\User Manual',
     'Renesas Electronics Corporation',
     'manual'),
]

# Spell checker configuration
spelling_lang = 'en_US'
spelling_word_list_filename = '../spelling_wordlist.txt'

# Copybutton configuration (keep comment lines, remove $/# prompts)
copybutton_prompt_text = r'^(?:[a-zA-Z0-9._-]+@[a-zA-Z0-9._-]+:[^$]*\$ |\$ )'
copybutton_prompt_is_regexp = True
copybutton_only_copy_prompt_lines = False
copybutton_remove_prompts = True