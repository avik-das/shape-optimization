all:
	# run repeatedly to resolve all references
	pdflatex shape-optimization.tex
	bibtex shape-optimization.aux
	pdflatex shape-optimization.tex
	pdflatex shape-optimization.tex

.PHONY: clean

clean:
	rm shape-optimization.aux
	rm shape-optimization.log
	rm shape-optimization.bbl
	rm shape-optimization.blg
	rm shape-optimization.pdf
