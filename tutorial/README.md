# INSLIB tutorials

Beginner-friendly tutorial to the INSLIB navigation filter.

* **[c_tutorial.md](c_tutorial.md)** - use the library from C: the three
  calls you actually need, a full runnable example, and the GNSS-free
  (lighthouse / UWB / mocap) path.
* **[python_tutorial.md](python_tutorial.md)** - the same filter from
  Python in a few lines, including install and the high-level `Navigator`.

## Optional: build PDFs

The Markdown renders to PDF via [pandoc](https://pandoc.org) and a LaTeX
engine:

```sh
sudo apt install pandoc texlive-xetex   # one-time (Debian/Ubuntu)
make            # -> c_tutorial.pdf, python_tutorial.pdf
make c          # just the C tutorial
make python     # just the Python tutorial
make clean
```

Prefer a different engine? `make PDF_ENGINE=pdflatex` or
`make PDF_ENGINE=tectonic`.
