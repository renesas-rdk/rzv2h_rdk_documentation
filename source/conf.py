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

extensions = ['myst_parser', 'sphinxcontrib.spelling', 'sphinx_copybutton']

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_favicon = '../renesas_favicon.png'

# -- Options for LaTeX output --------------------------------------------------

latex_elements = {
    'figure_align': 'H',  # Force all figures to stay in place (no floating)
}

latex_documents = [
    ('index', 'WS125_Robotic_Development_Kit_User_Manual.tex',
     'RZ/V2H Robotic Development Kit\\\\User Manual',
     'Renesas Electronics Corporation', 'manual'),
]

# Spell checker configuration
spelling_lang = 'en_US'
spelling_word_list_filename = '../spelling_wordlist.txt'

# Copybutton configuration
copybutton_prompt_text = r'^(?:[a-zA-Z0-9_-]+@[a-zA-Z0-9_-]+:[^$#]*[#$]\s|[#$]\s)'
copybutton_prompt_is_regexp = True
copybutton_only_copy_prompt_lines = False
copybutton_remove_prompts = True