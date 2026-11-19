# IDSC Thesis Writing Guide (for Claude)

Condensed from `docs/IDSCreport_v1.8.0/` (IDSCreport class v1.8.0, Aug 2023, and the 98-page
`IDSCreport_HOWTO.pdf`). Use this when writing or editing the **bachelor thesis** report of the
POLARIS mission planner. The rules below are IDSC (Prof. Onder group) standards; where something
is marked *mandatory* it is not a matter of taste.

---

## 1. Template setup

**Files** (`IDSCreport_v1.8.0/Template/`):

```
report.tex                 root file (preamble = metadata + abstract)
bibliography.bib           references
chapters/*.tex             one file per chapter / appendix, \input from report.tex
docstyle/                  IDSCreport.cls, logos, bibtexstyle/, packages/  (do not modify)
img/                       figures (create it)
report_as_it_should_look_like.pdf
```

**Root file skeleton for this thesis:**

```latex
% !TeX spellcheck = en_US
% !TeX encoding = UTF-8
\makeatletter\def\input@path{{./docstyle/}}\makeatother   % bug workaround, keep
\documentclass[bt]{IDSCreport}          % bt = Bachelor's thesis
% extra packages / \newacronym / \newsym go HERE, right after \documentclass

\title{Dynamics- and Tether-Aware Mission Planner for the POLARIS AUV}
\subtitle{...}                          % never put the subtitle inside \title
\author{Noel Bühler}
\ethid{..-...-...}
\semester{HS 2026}                      % or FS
\email{...@student.ethz.ch}
\supervision{Name\\ Prof. Dr. Name}     % separate supervisors with \\
\identification{IDSC-XX-YY-ZZ}
\date{<submission date>}
\keywords{keyword one, keyword two}
\bibliography{bibliography}             % set in the PREAMBLE, not at the end

\begin{abstract} ... \end{abstract}    % also: acknowledgment, preface, copyrightpage (preamble!)

\begin{document}
\input{chapters/introduction}
...
\appendix
\input{chapters/appendixA}
\end{document}
```

- Front-matter environments (`abstract`, `acknowledgment`, `preface`, `copyrightpage`) are written
  **in the preamble**, before `\begin{document}`; the class places them itself.
- Do **not** call `\maketitle`, `\tableofcontents`, `\bibliographystyle`, `\printbibliography`,
  `\cleardoublepage` before chapters: all automatic. `\maketitle` triggers a warning.
- Every chapter file begins with the magic comments
  `% !TeX spellcheck = en_US`, `% !TeX encoding = UTF-8`, `% !TeX root = ../report.tex`.

**Class options:** `bt` (Bachelor's thesis), `english` (default) / `german`, `teamwork`
(then `\authorA`/`\authorB` …), `noinfopage`, `nodeclaration`, `compact` (no blank pages, good
for proofreading drafts), `notoc`, `draft`, `article`.

**Build:** use latexmk with the PDF chain:

```
latexmk -pdf -silent -shell-escape -synctex=1 -output-directory=build report.tex
```

Recipe it automates: pdflatex → bibtex → (makeindex) → pdflatex → pdflatex. The HOWTO also
mentions `-r docstyle/nomenclature_latexmkrc`, but **that file is not in this copy of the
template**; the class uses `\printnoidxglossary` (no external indexer needed), so it should work
without it. Verify the nomenclature appears after the first full build.

**What the class already loads** (do not load again): amsmath, amssymb, amsthm, mathtools,
mathabx, commath, siunitx, xfrac, mhchem, textcomp, graphicx, float, pgfplots, booktabs, array,
multirow, xltabular, algorithm + algpseudocode, listings (+ styles `plaincode`, `matlabcode`),
enumitem, csquotes, ragged2e, fancyvrb, hyperref, url, cleveref (via `cleverefequations`, equation
refs in parentheses), glossaries-extra, parskip (no paragraph indent, vertical space instead),
placeins, caption, nowidow, microtype, lmodern, xcolor, xspace, geometry (150×229 mm text).
`epstopdf`, `circuitikz`, `subcaption` etc. are **not** loaded; add them after `\documentclass`.

---

## 2. Report structure

Order enforced by the class: Title page · (Copyright) · **Abstract + keywords** · (Acknowledgment) ·
Table of contents · (Preface) · Nomenclature · **Chapters** · (Appendices) · Bibliography ·
Information sheet with declaration of originality (must be **signed** by the student).

**Chapters (typical):**

1. **Introduction**: Motivation → Objective → Context (POLARIS project and how this thesis
   fits, incl. partner NMPC thesis) → State of research (literature, why existing solutions are
   insufficient) → Structure of the report (each chapter and why in this order).
2. **Theoretical part**: 2–3 chapters; starts with rigorous problem analysis, then the method.
3. **Results and discussion**: apply the theory (simulation and/or measurement data); always
   comment and interpret; the author is expected to be *the expert* by the end.
4. **Conclusion**: contributions + outlook (optionally limitations in between).

**Abstract:** 10–20 sentences. ~1/3 broad problem → specific issue; ~1/3 methods; ~1/3 results,
contribution, limitations.

**Chapter intro:** each chapter in the HOWTO opens with a short italic summary paragraph of what
the chapter does (style worth copying).

**Recommended writing order:** outline first → results & discussion → theoretical part (keep
only what the results need) → introduction → abstract & conclusion last. Around half-time,
agree on a thesis statement with the supervisor.

---

## 3. Language and style

- **American English** (decided by Noel, Nov 2026), as in the HOWTO: *-ize, -yze, -or, -er,
  maneuver, modeling, traveled, localization, behavior, fiber, meter, neighbor*.
- **Present tense** for everything not tied to a specific time; past only for specific past
  events (e.g. "the Lake Zurich campaign in 2025 showed …").
- **Active voice.** Avoid "It can be seen that …", "One can …". Write "Figure 3 shows …".
- **Address the broadest audience:** simplest explanation, short sentences, explain any jargon
  before using it. No tech slang.
- **Precision:** quantify claims ("RMSE of 0.45 m", not "accurate"). Round numbers to their
  actual precision.
- **No statements of judgment** ("good", "simple", "useful", "novel") about own work, except
  in the introduction (state of the art) and the conclusion. Argue instead.
- **Acronyms sparingly.** Only for widely known or very frequent terms; never invent new
  acronyms for own methods in a student report. Define via `\newacronym`, print via `\ac`.
- **Abbreviations** avoided except e.g., i.e., etc. Never start a sentence with e.g./i.e.
- **Parentheses and footnotes sparingly** (German habit). If info is essential → put in text;
  if not → delete. Footnote only if neither applies; it is a full sentence (capital, period),
  placed directly after the word (no space) or after the sentence period.

**Top grammar rules (from the HOWTO):**

1. *that* = restrictive (no comma); *which* = non-restrictive (with comma).
2. Transition words at sentence start take a comma ("However, …"); avoid them mid-sentence.
3. Hyphenate compound adjectives (*model-based, well-known, so-called, high-speed*); never
   after *-ly* adverbs. See list: nonlinear, online, off-line, e-mail, test bench, website, web page.
4. Don't elide in series: "the heat equation and the mass equation", not "heat and mass
   equations"; no suspended hyphens.
5. "yield to" does not exist (use "lead to" / "yield sth."); "allow to" needs an object.
6. **Serial (Oxford) comma**: "a, b, and c".
7. Comma between two independent clauses joined by and/or/but (not if subject is dropped).
8. Comma after a leading dependent clause.
9. "e.g.," is followed by a comma; "i.e." usually is not.
10. Adverb placement: mid position between subject and main verb / after first auxiliary.

**Capitalization:** sentence case in text. **Headings, table captions and names in title case**
(U.S. GPO style: capitalize all words except *a, an, the, at, by, for, in, of, on, to, up, and,
as, but, or, nor*).

---

## 4. LaTeX conventions (IDSC guidelines)

### Headings
`\chapter`, `\section`, `\subsection` (all in TOC), plus `\paragraph{Title}` for run-in bold
headings. Title case. No `\subsubsection`-depth needed normally.

### Emphasis and fonts
- `\emph{}` for italics (not `\textit`, except where italic is part of a name).
  Italicize a key term only on first occurrence.
- `\enquote{}` for quotation marks (csquotes), for scare quotes / so-called terms.
- No bold, small caps, etc. for emphasis.
- Products, companies, software (ROS 2, Gazebo, ArduSub, Matlab, Python): **plain roman**, no ™.
- Code, commands, packages, ROS topics/nodes, functions: `\texttt{}`.
- File names and paths: `\texttt{}`. File extensions: roman capitals (PDF, YAML, CSV).
- Titles of books/journals/reports: italic; articles/chapters/theses/web pages: quotes; websites,
  software, conferences: neither (see HOWTO Table A.5).

### Cross-references and citations
- `\label{}` **after** `\chapter/\section` and **after** `\caption` in floats; in equations just
  before the end.
- Reference with `\cref{}`; `\Cref{}` at sentence start. Never type "Figure~\ref{}".
- Label prefixes (mandatory convention): `chp:`, `sec:` (also subsections), `fig:`, `tab:`,
  `alg:`, `eq:`. E.g. `fig:VelocityProfile`.
- Citations: **IEEE style is mandatory** (class sets IEEEtran). `\cite{key}` with a non-breaking
  space: `Pure Pursuit~\cite{Coulter:1992}`. The sentence must be grammatical without the
  citation; never start a sentence with `[3]`. Name authors in text if giving them credit.

### Bibliography (`bibliography.bib`)
- Cite key: `Lastname:YEAR`, plus two random letters if ambiguous; student reports get suffix
  `bt`/`sp`/`mt`/`sm`, dissertations `diss` (e.g. `Ritter:2014mt`).
- Protect capitalization in titles with braces: `{AUV}`, `{ROS}`, `{NMPC}`.
- Fill in `language = {english}`.
- Entry types: `@article` (author, title, journal [IEEEabrv abbreviation], volume, number, pages,
  year), `@book`, `@booklet`, `@incollection`, `@inproceedings` (booktitle, publisher, pages),
  `@mastersthesis` for **all student reports** (with `type = {Bachelor's thesis}`,
  `school = {ETH Zurich, ...}`, `note = {{IDSC-..}}`), `@phdthesis`, `@manual` (software
  docs, versions), `@techreport` (e.g. the POLARIS final report), `@webpage` (url, month, year),
  `@electronic` (software distributed online without manual, e.g. nav2, ArduSub, with version
  in `note`).
- If using a reference manager, keep a separate auto-generated .bib and a manual .bib.

### Units (siunitx, mandatory)
- Always `\qty{0.45}{\meter}`, `\unit{\meter\per\second}`, `\num{10000}`,
  `\qtyrange{1}{2}{\meter}`, `\numrange{}{}`, `\ang{90}` or `\qty{90}{\degree}`,
  `\qty{23}{\celsius}`, `\qtyproduct{20 x 20}{\meter}`.
- **Percent without siunitx**: `5\%` (no space).
- Unit of a variable: brackets on the variable, `[v] = \unit{\meter\per\second}`.
- Written-out style ("seven seconds", "12~seconds") → no siunitx, use `~`. Ranges with en dash `--`.
- Custom units via `\DeclareSIUnit` in the preamble (e.g. `\DeclareSIUnit{\knot}{kn}` if needed).
- Axis labels / table headers: quantity / unit (SI guide), e.g. "Velocity / m/s", or pgfplots
  `units` library with `unit markings=slash space`. Be consistent across all figures.

### Math
- Variables italic single letters; descriptive sub/superscripts upright:
  `t_\mathrm{f}`, `v_\mathrm{max}`, `x_\mathrm{SoC}`; index variables italic (`x_k`).
- Operators upright: `\sin`, `\exp`, `\arg\min`; define new ones with `\DeclareMathOperator`.
  Differential d upright: commath `\dod{f}{x}`, `\dif x`, `\pd`.
- Transpose: `\vec{a}^\textsf{T}`. Sets: `\{x \mid x > 0\}`. Maps: `f \colon x \to x^2`. Dots: `\dots`.
- Constants e, π, i: italic (IDSC deviates from ISO). Numbers upright.
- Environments: only `equation` (one number) and `align` (one number per line);
  `equation`+`aligned` for one number on several lines; `*` versions or `\nonumber` to suppress.
- **Equations are part of the sentence: punctuate them** (comma/period inside the display).
  No punctuation inside inline `$...$`; each list variable in its own `$...$`.
- `cases`, `pmatrix` (vectors) / `bmatrix` (matrices), `\left( \right)` for auto-sized brackets.

### Floats
- **Figures**: caption **below**; `\centering`, `\includegraphics[width=..]{img/..}`; caption in
  full sentences, figure + caption self-explanatory; one message per figure; meaningful legend
  and axis labels **with units**; ~12 pt fonts in final size; every figure referenced in text, and
  the text must stand alone even if the float moves.
- **Tables**: caption **above**, in title case; booktabs only (`\toprule`, `\midrule`,
  `\bottomrule`, `\addlinespace`), **no vertical lines**; numbers in `S` columns
  (`table-format=2.1`), non-numbers in `S` cells wrapped in `{}`; units in header.
  Mixed-unit parameter table: `lS@{\,}l` with value and `\unit{}` columns.
- **Algorithms**: `algorithm` float + `algorithmic` (algpseudocode syntax: `\Function`,
  `\For`, `\State`, `\If`, `\Comment`). Prefer pseudocode over raw code in the main text
  (good fit for nearest-neighbour / 2-opt / waypoint generation).
- Placement: `[htbp]`, `!` to force; `\FloatBarrier` (placeins) to stop floats drifting
  past a section.

### Plots / graphics
- Vector graphics only (PDF/EPS); never screenshots (except to show a GUI).
- Best quality: pgfplots/TikZ with data in external files, fonts typeset by LaTeX. For Python
  (matplotlib) plots: export PDF (or PGF) with font sizes matched to the 10 pt document and the
  final width, or use `tikzplotlib`-style export; keep consistent styling across figures.
- Inkscape: export EPS (+ "omit text, create LaTeX file") and include with the class commands
  `\includeinkscape` / `\includelabeledeps`.
- TikZ externalization (`\tikzexternalize`) to speed up compiles if many plots.

### Lists, code, symbols
- `itemize` / `enumerate` / `description`; tune spacing via enumitem options (`nosep`, `itemsep`).
- Code: `\begin{lstlisting}[style=plaincode,xleftmargin=1em]` or `\lstinputlisting`; whole
  files only in the appendix.
- Text symbols in text mode: `\textpm`, `\texttimes`, `\textrightarrow`, `\bs`, `\texttilde`.
- `\LaTeX` and `\TeX` do not need `{}` in this class.

### Nomenclature (glossaries-extra, auto-generated, only used entries appear)
- Acronyms in preamble: `\newacronym{auv}{AUV}{autonomous underwater vehicle}`
  (`longplural=`, `description=`, `prefix=`/`prefixfirst=` for a/an options).
- Use: `\ac` (first use long + short), `\acs`, `\acl`, `\acf`, plurals `\acp` …, capitalized `\Ac` …
  In **captions and headings** use `\glsfmtshort`, `\glsfmtlong`, `\glsfmtfirst` (no `\ac`).
- Symbols: `\newsym[description={velocity}, unit=\unit{\meter\per\second}, sort=v]{velocity}{\ensuremath{v}}`;
  print with `\gls{velocity}`; Greek letters sort as `zzz <latin>` (e.g. ψ → `sort=zzz y`);
  use `\texorpdfstring` for symbols that may appear in headings.
- `\glsresetall` after the abstract so acronyms are spelled out again in chapter 1.

---

## 5. Checklist before handing in a draft

- [ ] Class option `bt`, all metadata fields filled, keywords set, declaration page signed.
- [ ] Abstract 10–20 sentences, problem → method → results/limitations.
- [ ] Present tense, active voice, no judgment words, no unexplained acronyms/jargon.
- [ ] Consistent English variant (AE or BE) throughout, incl. figures.
- [ ] All quantities via siunitx, percent as `\%`, sensible precision.
- [ ] Every figure/table/equation referenced with `\cref`, labels with correct prefixes.
- [ ] Figure captions below, table captions above (title case), booktabs, no vertical lines.
- [ ] Equations punctuated; upright descriptive subscripts and operators.
- [ ] IEEE bibliography complete, cite keys `Name:YEAR`, capitalization protected.
- [ ] Footnotes/parentheses minimized; serial commas; title-case headings.
- [ ] Clean latexmk build: no undefined references, no overfull boxes.

---

## 6. Other material in the folder

`Documentation/` holds generic package manuals (siunitx, cleveref, booktabs, enumitem,
glossaries, pgfplots, listings, algorithms, mathtools, TikZ, symbol lists, "Not So Short
Introduction to LaTeX2e", IEEEtran_bst_HOWTO). Consult these for details; nothing
IDSC-specific beyond what is summarized above.

Cite the template itself if used: `@manual{IDSCreportClass, ...}` (already in the template's
`bibliography.bib`).
