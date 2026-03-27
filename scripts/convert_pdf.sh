#!/bin/bash

### install asciidoc
#sudo apt update
#sudo apt -y install ruby-full
#sudo apt -y install graphviz
#sudo apt -y install rubygems
#sudo gem install public_suffix -v 5.1.1
#sudo gem install bundler -v 2.4.22
#sudo gem install css_parser -v 1.17.1
#sudo gem install asciidoctor
#sudo gem install asciidoctor-pdf --pre
#sudo gem install asciidoctor-pdf-cjk
#sudo gem install asciidoctor-diagram

### Variables that need to be changed
PACKAGE_VER=${PACKAGE_VER:-"1.0"}

### fixed variables
SRC_ADOC=$1
DST_PDF=$2
THEMES_DIR=docs/data/themes
FONTS_DIR=docs/data/fonts
THEMES_FILE=tmp
DOC_NUMBER="R-----"
REL_LOGO="renesas_logomark_l_clearspace.jpg"
REV_DATE=${REV_DATE:-"Mar.31.26"} # CHANE ME if needed

### Modify the themes of PDF
cp  ${THEMES_DIR}/sample-theme.yml  ${THEMES_DIR}/${THEMES_FILE}-theme.yml
sed -i -e "s/{PACKAGE_VER}/${PACKAGE_VER}/g" ${THEMES_DIR}/${THEMES_FILE}-theme.yml
sed -i -e "s/{DOC_NUMBER}/${DOC_NUMBER}/g"   ${THEMES_DIR}/${THEMES_FILE}-theme.yml
sed -i -e "s/{REL_LOGO}/${REL_LOGO}/g"       ${THEMES_DIR}/${THEMES_FILE}-theme.yml
sed -i -e "s/{REV_DATE}/${REV_DATE}/g"       ${THEMES_DIR}/${THEMES_FILE}-theme.yml

### Generate PDF
asciidoctor-pdf --verbose --trace -a pdf-theme=${THEMES_FILE} -a pdf-themesdir=${THEMES_DIR} -a pdf-fontsdir=${FONTS_DIR} ${SRC_ADOC} -o ${DST_PDF}
