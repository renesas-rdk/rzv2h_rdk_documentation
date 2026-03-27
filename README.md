# RZV2H Robotic Development Kit User Manual

## Overview
This repository provides a ready-to-use User Manual for the **RZ/V2H Robotic Development Kit (RDK)**.
It also includes a generated **PDF file** for offline viewing.

The documentation is built with [Sphinx](https://www.sphinx-doc.org/en/master/) using a theme provided by [Read the Docs](https://about.readthedocs.com/?ref=app.readthedocs.org).

## How to use

Access the Online Manual which is available at:

🤖 [RZ/V2H Robotic Development Kit User Manual (Online)](https://renesas-rdk.github.io/rzv2h_rdk_documentation/)

For offline use, download the generated PDF after each build at: `docs/pdf/WS125_Robotic_Development_Kit_User_Manual.pdf`

The **RZ/V2H Robotic Development Kit Hardware Manual** is also available in PDF format at: `docs/pdf/WS125-V2HRDKREFZ-Hardware-Manual.pdf`

---

## Building the documentation locally

### Prerequisites
To build this, you need to install the following:

- **Docker**

Refer to the official [Docker installation guide](https://docs.docker.com/engine/install/) to install it on your machine.


### Setup the Docker

1. Clone the repository and change the workspace to the cloned directory:

    Install ``git-lfs`` if not available (must be installed **before** cloning):

    ```bash
    sudo apt update
    sudo apt install git-lfs
    ```

    Clone the repo (LFS files will be pulled automatically):

    ```bash
    git clone https://github.com/renesas-rdk/rzv2h_rdk_documentation.git
    cd rzv2h_rdk_documentation
    ```

2. Build the Docker image:

    ```bash
    docker build -f docker/image/Dockerfile --build-arg user=$(id -un) --build-arg uid=$(id -u) -t rdk_documentation .
    ```

3. Create the container:

   Mount the current documentation repository inside the Docker container.

    ```bash
    docker run -dt --name rdk_doc_build -v $(pwd):/tmp/doc_repository rdk_documentation
    ```

4. Enter the Docker environment (for subsequent sessions):

    ```bash
    docker exec -it rdk_doc_build bash
    ```

    Or combine it with the **Dev Containers** extension in **VS Code** for a more convenient development experience.

### Build the HTML

Run the following command after you finish modifying the documentation:
```bash
make html
```

The result will be located at: `build/html/index.html`

### Build Multiple Versions (sphinx-polyversion)

Build all configured versions (release tags + main/master) with:

```bash
sphinx-polyversion poly.py build/html
```

(Optional) For convenience, you may use Visual Studio Code (VSCode) and the Live Server extension to view the documentation, but this is not required for building or viewing the documentation.

**Steps:**
1. Open the project folder in VSCode.

2. Install the Live Server extension:

    - Open the Extensions Marketplace (Ctrl+Shift+X).

    - Search for `Live Server by Ritwick Dey`.

    - Click Install.

3. In the VSCode Explorer, navigate to: `build/html/index.html`

4. Right-click the file and select "Open with Live Server".

5. A browser window will open showing your documentation.
Any changes you make and rebuild (make html) will be visible after the `make html` command is executed.

*Tip:* If auto-refresh does not work, clean the build folder (`make clean`), rebuild the HTML and manually refresh the browser tab.

### Build the PDF

Generate the PDF with the conversion script:

```bash
DOCS_VERSION=${DOCS_VERSION:-1.0}
python3 scripts/convert.py \
  --output_filename WS125_Robotic_Development_Kit_User_Manual.pdf \
  --release_version "${DOCS_VERSION#v}" \
  --rev_date "$(date '+%b.%d.%y')"
```

The output will be located at: `output/WS125_Robotic_Development_Kit_User_Manual.pdf`

Optional manual override:

```bash
python3 scripts/convert.py \
  --output_filename WS125_Robotic_Development_Kit_User_Manual.pdf \
  --release_version 1.0 \
  --rev_date Mar.31.26
```

### Spell Check

Run the following command to check for spelling errors:

```bash
make spelling
```

This command will generate a list of misspelled words. If you believe a word should be ignored by the spell checker, add it to the [spelling_wordlist.txt](spelling_wordlist.txt) file.

### VSCode Task Configuration

To simplify the build process, you can use the preconfigured VS Code tasks for HTML, spelling, and PDF generation.

1. Open the project folder in VSCode.
2. Open the Command Palette (Ctrl+Shift+P) and type `Run Task`, then select it.
3. Choose the desired task from the list:
   - `Build HTML`: Generates the HTML documentation (developer version).
   - `Build HTML (polyversion)`: Generate HTML documentation for all configured versions (max 3 versions) using sphinx-polyversion (release versions only).
   - `Build Spelling`: Performs a spell check on the documentation.
   - `Build PDF (convert.py)`: Generates the PDF via `scripts/convert.py`.
   - `Build All (spelling, html, pdf)`: Runs spelling, HTML build, and PDF generation sequentially.
