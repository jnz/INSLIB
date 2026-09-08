# Render the INSLIB tutorials to PDF with pandoc + a LaTeX engine.
#
#   make            # build all PDFs
#   make c          # just the C tutorial
#   make python     # just the Python tutorial
#   make clean      # remove the PDFs
#
# Requirements: pandoc and a LaTeX engine. On Debian/Ubuntu:
#   sudo apt install pandoc texlive-xetex
# Override the engine if you prefer another one:
#   make PDF_ENGINE=pdflatex
#   make PDF_ENGINE=tectonic

PANDOC     ?= pandoc
PDF_ENGINE ?= xelatex

SOURCES := c_tutorial.md python_tutorial.md
PDFS    := $(SOURCES:.md=.pdf)

# A readable, self-contained PDF: numbered table of contents, sane margins,
# coloured links, syntax-highlighted code.
PANDOC_FLAGS := \
	--pdf-engine=$(PDF_ENGINE) \
	--toc --toc-depth=2 \
	--number-sections \
	--highlight-style=tango \
	-V geometry:margin=1in \
	-V colorlinks=true \
	-V linkcolor=blue \
	-V urlcolor=blue \
	-V fontsize=11pt

.PHONY: all c python clean
all: $(PDFS)

c: c_tutorial.pdf
python: python_tutorial.pdf

%.pdf: %.md
	$(PANDOC) $(PANDOC_FLAGS) -o $@ $<

clean:
	rm -f $(PDFS)
