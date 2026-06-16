.PHONY: all clean mermaid

all: mermaid
	docker run --rm -v $$(pwd):/workdir -w /workdir texlive/texlive:latest latexmk -pdf -interaction=nonstopmode -file-line-error main.tex

mermaid:
	@if ls diagrams/*.mmd 1> /dev/null 2>&1; then \
		chmod a+w images/; \
		for f in diagrams/*.mmd; do \
			filename=$$(basename -- "$$f"); \
			name="$${filename%.*}"; \
			echo "Génération de l'image Mermaid pour $$f..."; \
			docker run --rm -v $$(pwd):/data minlag/mermaid-cli -i "/data/$$f" -o "/data/images/$$name.pdf"; \
		done \
	fi

clean:
	docker run --rm -v $$(pwd):/workdir -w /workdir texlive/texlive:latest latexmk -C
	rm -f main.bbl main.run.xml
	rm -f images/*.pdf
