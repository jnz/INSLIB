# Makefile for ins's documentation: the hand-written design doc (LaTeX)
# and the Doxygen API reference (@brief/@param comments in src/ + the
# Python C ABI shim, see Doxyfile) -- two independent artifacts under one
# `make doc`.
#
# Usage:
#   make            build INSLIB_manual.pdf AND the doxygen/ API reference
#   make doxygen    build only the doxygen/ API reference (html + latex)
#   make clean      remove LaTeX build artifacts (keeps the PDF)
#   make distclean  remove all build artifacts, the PDF and doxygen/
#
# Prefers latexmk (handles reruns for TOC/references automatically);
# falls back to three pdflatex passes if latexmk is not installed.
# `doxygen` needs the doxygen binary (graphviz optional, for call graphs).

DOC     := INSLIB_manual
LATEXMK := $(shell command -v latexmk 2>/dev/null)

.PHONY: all clean distclean doxygen doxygen-clean

all: $(DOC).pdf doxygen

$(DOC).pdf: $(DOC).tex
ifneq ($(LATEXMK),)
	latexmk -pdf -interaction=nonstopmode -halt-on-error $(DOC).tex
else
	pdflatex -interaction=nonstopmode -halt-on-error $(DOC).tex
	pdflatex -interaction=nonstopmode -halt-on-error $(DOC).tex
	pdflatex -interaction=nonstopmode -halt-on-error $(DOC).tex
endif

# API reference (doxygen/html, doxygen/latex) from the src/ headers'
# @brief/@param comments. Optional: `make -C doxygen/latex` (needs
# latexmk/pdflatex, same as above) turns the LaTeX refman into a PDF.
doxygen:
	doxygen Doxyfile

doxygen-clean:
	rm -rf doxygen

clean:
ifneq ($(LATEXMK),)
	latexmk -c $(DOC).tex
else
	rm -f $(DOC).aux $(DOC).log $(DOC).out $(DOC).toc $(DOC).fls \
	      $(DOC).fdb_latexmk
endif

distclean: clean doxygen-clean
	rm -f $(DOC).pdf
