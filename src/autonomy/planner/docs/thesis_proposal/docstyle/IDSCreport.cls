% IDSCreport.cls LaTeX document class for reports at IDSC (ETH Zurich)
% Copyright (C) 2020  Andreas Ritter

% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.


% ==============================================================================
% Ignore certain warnings
% ==============================================================================
\RequirePackage{silence}
\WarningFilter{latex}{You have requested package} % ignore warnings of latex for packages in subfolders
\WarningFilter{latex}{You have requested document class} % ignore warnings of latex for document class in subfolders
\WarningFilter{hyperref}{Token not allowed in a PDF string} % ignore warnings of hyperref when pdf properties are set



% ==============================================================================
% Main class definition of IDSCreport.cls
% ==============================================================================
\ProvidesClass{IDSCreport}[2020/22/12 by Andreas Ritter (ETH Zurich)]
\NeedsTeXFormat{LaTeX2e}

\typeout{-- IDSCreport.cls LaTeX document class for reports at IDSC (ETH Zurich)}
\typeout{See the "IDSCreport_HOWTO" manual for usage information.}
\typeout{IDSCreport.cls Copyright (C) 2020  Andreas Ritter, ETH Zurich}
\typeout{This program comes with ABSOLUTELY NO WARRANTY; for details see license.txt.}
\typeout{This is free software, and you are welcome to redistribute it}
\typeout{under certain conditions; see license.txt for details. ---}



% ==============================================================================
% Define boolean variables
% ==============================================================================
\newif\if@isgerman\@isgermanfalse
\newif\if@isgeneric\@isgenericfalse
\newif\if@isinfopage\@isinfopagetrue
\newif\if@isdeclaration\@isdeclarationtrue
\newif\if@iscompact\@iscompactfalse
\newif\if@istableofcontents\@istableofcontentstrue
\newif\if@isarticle\@isarticlefalse
\newif\if@issubtitle\@issubtitlefalse
\newif\if@isteamwork\@isteamworkfalse
\newif\if@iscopyrightpage\@iscopyrightpagefalse
\newif\if@ispreface\@isprefacefalse
\newif\if@isabstract\@isabstractfalse
\newif\if@isacknowledgment\@isacknowledgmentfalse



% ==============================================================================
% Declare class options and load underlying class report
% ==============================================================================
% Language (german or english)
\DeclareOption{german}{\@isgermantrue}
\DeclareOption{english}{\@isgermanfalse}

% Thesis type
\DeclareOption{sm}{\gdef\@type{Studies on Mechatronics}}
\DeclareOption{bt}{\gdef\@type{\if@isgerman Bachelorarbeit\else Bachelor's Thesis\fi}}
\DeclareOption{sp}{\gdef\@type{\if@isgerman Semesterarbeit\else Semester Project\fi}}
\DeclareOption{mt}{\gdef\@type{\if@isgerman Masterarbeit\else Master's Thesis\fi}}
\DeclareOption{is}{\gdef\@type{\if@isgerman Praktikumsbericht\else Internship Report\fi}}

% Teamwork
\DeclareOption{teamwork}{\@isteamworktrue}

% Infopage and declaration
\DeclareOption{noinfopage}{\@isinfopagefalse}
\DeclareOption{nodeclaration}{\@isdeclarationfalse}
\DeclareOption{compact}{\@iscompacttrue}

% Further options
\DeclareOption{notoc}{\@istableofcontentsfalse}
\DeclareOption{article}{\@isarticletrue}

% Passing options to the underlying report class
\DeclareOption{draft}{\PassOptionsToClass{\CurrentOption}{report}}

% Warning for unkonwn options, set default options and process options
\DeclareOption*{\ClassWarning{IDSCreport}{Unknown option \CurrentOption.}}
\ExecuteOptions{english}
\ProcessOptions\relax

% Load class report with the following options and the ones passed by \PassOptionsToClass
\if@iscompact
\if@isarticle
\LoadClass[10pt,a4paper,fleqn]{article}
\else
\LoadClass[10pt,a4paper,fleqn]{report}
\fi
\else
\if@isarticle
\LoadClass[10pt,twoside,a4paper,fleqn]{article}
\else
\LoadClass[10pt,twoside,a4paper,fleqn]{report}
\fi
\fi

% Check if any specific document type is requested. Else set to generic report
\@ifundefined{@type}{
\@isgenerictrue
\@isinfopagefalse
}



% ==============================================================================
% Load packages
% ==============================================================================
% Special packages required for the implementation of this class
\RequirePackage{environ} % simplifies the definition of new environments
\RequirePackage{ifpdf}
\if@iscompact
\RequirePackage{draftwatermark}
\fi

% Language
\if@isgerman
\RequirePackage[ngerman]{babel} % defines that document is written in German using the new spelling rules
\fi
\RequirePackage[T1]{fontenc} % ensure that type 1 fonts are used (these are full vector-based)
\RequirePackage[utf8]{inputenc} % using utf8 enables to write umlauts in German directly (without the \")

% General
\RequirePackage{fancyhdr}
\ifpdf % pdflatex
\RequirePackage[pdftex,textwidth=150mm,textheight=229mm,hcentering,tmargin=4cm]{geometry}
\else % latex (eps-dvi-pdf)
\RequirePackage[dvipdfm,textwidth=150mm,textheight=229mm,hcentering,tmargin=4cm]{geometry}
\fi % end \ifpdf
\RequirePackage[nottoc]{tocbibind} % include the bibliography in the table of contents
\RequirePackage{parskip} % manages the two lengths \parskip and \parindent
\RequirePackage{placeins} % preventing floats from floating past a \FloatBarrier
\RequirePackage{caption} % enables caption customization
\RequirePackage[all]{nowidow} % removes orphans and widows

% Font
\RequirePackage{lmodern} % use improved fonts compared to the standard latex fonts
\RequirePackage{anyfontsize} % for proper scaling of font sizes not supported by default
\RequirePackage{xcolor} % for colored text with command \color{<color>}{<text>}
\RequirePackage{xspace} % define commands that appear not to eat spaces
\RequirePackage{microtype} % Improves the capabilities for typesetting, solving most under / overfull box issues.

% Graphics
\RequirePackage{graphicx} % include graphics
\RequirePackage{float} % adds feature to place floats where defined via h! or H modifiers
\RequirePackage{pgfplots} % adds native plotting features to LaTeX
\pgfplotsset{compat=1.12} % ensures compatibility

% Math
\RequirePackage{amsmath} % provides most of the mathematical features of AMS-TEX
\RequirePackage{amssymb} % provides an extended symbol collection and loads amsfonts
\RequirePackage{amsthm} % helps to define theorem-like structures
\RequirePackage{mathtools} % fixes various deficiencies of amsmath and standard LATEX
\RequirePackage{mathabx} % provides even more mathematical symbols
\RequirePackage{commath} % for nice derivative operators

% Symbols and measurements of units
\RequirePackage{textcomp} % provides an extensive list of text symbols
\RequirePackage{xfrac} % provides the command \sfrac to print nice d/n fractions
\RequirePackage[version=3]{mhchem} % for typesetting chemical molecular formulae and chemical equations
\RequirePackage{siunitx} % scientific units, numbers, and quantities


% Tables
\RequirePackage{multirow} % construction for table cells that span more than one row
\RequirePackage{array} % extends options for tabular environment
\RequirePackage{xltabular} % implements xltabular, a combination of longtable and tabularx used for the glossaries
\RequirePackage{booktabs} % enhances the quality of tables and provides extra commands

% Environments
\if@isarticle
\RequirePackage{algorithm} % for floating algorithm environment
\else
\RequirePackage[chapter]{algorithm} % for floating algorithm environment
\fi
\RequirePackage{algpseudocode} % for pseudocode
\RequirePackage{fancyvrb} % allows to indent the verbatim environment
\RequirePackage{enumitem} % allows to customize lists such as itemize and enumerate
\RequirePackage{ragged2e} % allows setting ragged text which is easy to configure
\RequirePackage{csquotes} % advanced facilities for inline and display quotations
\RequirePackage{listings} % advanced typesetting of source code

% Internet addresses and interactive links
\RequirePackage[hyphens]{url}
\ifpdf % pdflatex
\RequirePackage[pdftex,pdfpagelabels,pdfencoding=auto,psdextra]{hyperref}
\else % latex (eps-dvi-pdf)
\RequirePackage{hyperref}
\RequirePackage[hyphenbreaks]{breakurl}
\fi % end \ifpdf

% Special packages that are not part of the TEX distribution
\RequirePackage{docstyle/packages/sourcecode}
\RequirePackage{docstyle/packages/specialgraphics}
\RequirePackage{docstyle/packages/cleverefequations}



% ==============================================================================
% Commands for appearance of specific words and symbols
% ==============================================================================
% Shorter names for known symbols
\newcommand\bs\textbackslash % define \bs
\renewcommand{\texttilde}{\nolinkurl{\~}{}\xspace} % define \texttilde

% Product names
\let\LaTeXtemp\LaTeX \renewcommand{\LaTeX}{\LaTeXtemp{}\xspace} % redefine \LaTeX
\let\TeXtemp\TeX \renewcommand{\TeX}{\TeXtemp{}\xspace} % redefine \TeX

% Physical units
\newcommand{\celsius}{\si{\degree}C}
\newcommand{\fahrenheit}{\si{\degree}F}


% ==============================================================================
% Settings for glossaries
% ==============================================================================
% Load the glossaries-extra package, which is internally loads the glossaries package
\RequirePackage[
acronyms, % define commands for acronyms
symbols, % define commands for symbols
shortcuts=all, % define shortcuts for acronyms and symbols
toc=false, % do not add the glossary lists to the table of contents
section=section, % the titles of the glossary lists should be on the same level as the sections
nopostdot, % do not add dots in the glossary lists
nonumberlist, % do not put usage number to the glossary lists
prefix, % loads the package glossaries-prefix
]{glossaries-extra}

% Define how glossaries are created and how the acronyms are printed
\makenoidxglossaries % Initilize with "Option 1" (see http://tug.ctan.org/macros/latex/contrib/glossaries/glossariesbegin.pdf)
\setabbreviationstyle[\acronymtype]{long-short} % First time, display it in the long form, then short

% Define a custom style for listing the acronyms
\newglossarystyle{acronymtable}{%
\setglossarystyle{long4col}%
\renewenvironment{theglossary}%
{\xltabular{\linewidth}{@{}lX}}%
{\endxltabular}%
\renewcommand*{\glossaryheader}{}% no header
\renewcommand*{\glsgroupheading}[1]{}% no group header
\renewcommand{\glossentry}[2]{% custom row entry
% Row defined as: ABBR    Description of the abbreviation
\glstarget{##1}{\glossentryname{##1}} & \Glsentrydesc{##1} \tabularnewline%
}
\renewcommand*{\glsgroupskip}{\relax}% no space between groups
}

% Define a custom style for listing the symbols
\newglossarystyle{symboltable}{%
\setglossarystyle{long4col}%
\renewenvironment{theglossary}%
{\xltabular{\linewidth}{@{}lXr@{\,}l}}%
{\endxltabular}%
\renewcommand*{\glossaryheader}{}% no header
\renewcommand*{\glsgroupheading}[1]{}% no group header
\renewcommand{\glossentry}[2]{% custom row entry
% Row: symb    Description of the symbol           magnitude unit
\glstarget{##1}{\glossentryname{##1}} & \Glsentrydesc{##1} & \glsmagnitude{##1} & \glsunit{##1}\tabularnewline%
}
\renewcommand*{\glsgroupskip}{\relax}% no space between groups
}

% Add custom fields for annotating symbols
\glsaddstoragekey{unit}{}{\glsunit}
\glsaddstoragekey{magnitude}{}{\glsmagnitude}

% Turn off expansion for a field, see https://tex.stackexchange.com/a/192351
\glssetnoexpandfield{unit}
\glssetnoexpandfield{magnitude}



% ==============================================================================
% Add metadata to resulting pdf document
% ==============================================================================
\AtBeginDocument{
\hypersetup{
bookmarksnumbered,
pdftitle={\@pdftitle\if@issubtitle: \@thepdfsubtitle\fi},
pdfauthor={\if@isteamwork \@pdfauthorA\xspace and \@pdfauthorB \else \@pdfauthor\fi},
pdfsubject={\@ifundefined{@type}{}{\@type{}\if@isgeneric\else \@identification\fi}},
pdfkeywords={\@ifundefined{@keywords}{}{\@keywords}}
}} % end hypersetup and end \AtBeginDocument



% ==============================================================================
% Document layout
% ==============================================================================
% Define settings for text layout
\frenchspacing % makes the sentence spacing single spaced
\bibliographystyle{docstyle/bibtexstyle/IEEEtran} % official IEEE transaction referencing style
\setcounter{tocdepth}{2} % Table of contents contains only chapters and sections
\captionsetup[table]{skip=10pt} % Increase space between caption and content of tables

% Define page clearance command
\if@isarticle
\newcommand{\@pagebreak}{\clearpage}
\else
\newcommand{\@pagebreak}{\cleardoublepage}
\fi

% Definition of page header
\if@isarticle
\else
\rhead[\nouppercase{\rightmark}]{\thepage}
\lhead[\thepage]{\nouppercase{\leftmark}}
\cfoot{}
\fi

% Watermark every page if it is in compact mode
\if@iscompact
\SetWatermarkAngle{0}
\SetWatermarkFontSize{1cm}
\SetWatermarkHorCenter{13cm}
\SetWatermarkVerCenter{4.5cm}
\SetWatermarkText{Compact mode}
\fi

% Beginning of the document
\AtBeginDocument{
\typesettitlepage
\restoregeometry
\pagenumbering{roman}
\@pagebreak
\if@isabstract
\typesetabstract
\else
\if@isgeneric
% generic report without abstract is fine.
\else
\ClassWarning{IDSCreport}{Abstract is missing}{}
\typesetabstractwarning
\fi
\fi
\@pagebreak
\if@isacknowledgment
\typesetacknowledgment
\fi
\if@istableofcontents
\tableofcontents
\fi
\@pagebreak
\if@ispreface
\typesetpreface
\fi
\@pagebreak
\typesetnomenclature
\@pagebreak
\pagestyle{fancy}
\if@isarticle
\renewcommand{\sectionmark}[1]{\markboth{#1}{}} % set the \leftmark
\fancyhf{}
\fancyhead[R]{} % predefined ()
\fancyhead[L]{\leftmark} % 1. sectionname
\fancyfoot[C]{\thepage}
\fancypagestyle{plain}{%
\fancyhf{}%
\renewcommand{\headrulewidth}{0pt}%
}
\fi
\pagenumbering{arabic}
} % end \AtBeginDocument

\AtEndDocument{
\@ifundefined{@thebibliography}{}{\originalbibliography{\@thebibliography}}
\if@isinfopage
\typesetinfopage
\fi % end @isinfopage
} % end \AtEndDocument



% ==============================================================================
% Define or redefine other commands
% ==============================================================================
% Commands for layouting the document
\newcommand{\setfontsize}[2]{\fontsize{#1}{#1}\selectfont #2 \setlength{\baselineskip}{1.2\baselineskip}}

% remove \\ from given input argument. It doesn't matter if the user types something like ("this   //   test" or "this//test")
\newcommand{\removelinebreaks}[1]{% comment symbol is required directly after line finishes
\renewcommand{\\}{\leavevmode\unskip\xspace}% comment symbol is required directly after line finishes
#1 % give the argument as output with the above changes
} % end of command \removelinebreaks

% Force chapters and parts to start on a new odd page
\if@isarticle
% do nothing
\else
\let\originalchapter\chapter \renewcommand{\chapter}{\cleardoublepage\originalchapter}
\fi
\let\originalpart\part \renewcommand{\part}{\cleardoublepage\originalpart}

% Make default layout commands obsolete
\renewcommand{\maketitle}{\ClassWarning{IDSCreport}{You cannot use this command}{}}



% ==============================================================================
% Define commands for document properties
% ==============================================================================
% General document properties
\renewcommand{\title}[2][\@empty]{
\renewcommand{\@title}{#2}
\newcommand{\@thetitle}{#2}
\newcommand{\@thetitletext}{\removelinebreaks{#2}}
\renewcommand{\@pdftitle}{\ifx\@empty#1\relax#2\else#1\fi}
} % end of command \title
\newcommand{\subtitle}[2][\@empty]{
\@issubtitletrue
\newcommand{\@subtitle}{#2}
\newcommand{\@subtitletext}{\removelinebreaks{#2}}
\newcommand{\@thepdfsubtitle}{\ifx\@empty#1\relax#2\else#1\fi}
} % end of command \subtitle
\renewcommand{\date}[1]{\newcommand{\@thedate}{#1}\renewcommand{\@date}{#1}}
\newcommand{\supervision}[1]{\newcommand{\@supervision}{#1}}
\newcommand{\identification}[1]{\newcommand{\@identification}{#1}}
\newcommand{\keywords}[1]{\newcommand{\@keywords}{#1}}
\newcommand{\type}[1]{\newcommand{\@type}{#1}}

% Bibliography
\let\originalbibliography\bibliography
\renewcommand{\bibliography}[1]{\newcommand{\@thebibliography}{#1}}

% Author information
\renewcommand{\author}[2][\@empty]{
\if@isgeneric
\newcommand{\@authors}{#2}
\else
\newcommand{\@student}{#2}
\fi
\renewcommand{\@pdfauthor}{\ifx\@empty#1\relax#2\else#1\fi}
} % end of command \author
\newcommand{\ethid}[1]{\newcommand{\@ethid}{#1}}
\newcommand{\semester}[1]{\newcommand{\@semester}{#1}}
\newcommand{\email}[1]{\newcommand{\@email}{#1}}

% Author A for team work
\newcommand{\authorA}[2][\@empty]{
\newcommand{\@studentA}{#2}
\newcommand{\@pdfauthorA}{\ifx\@empty#1\relax#2\else#1\fi}
} % end of command \authorA
\newcommand{\ethidA}[1]{\newcommand{\@ethidA}{#1}}
\newcommand{\semesterA}[1]{\newcommand{\@semesterA}{#1}}
\newcommand{\emailA}[1]{\newcommand{\@emailA}{#1}}

% Author B for team work
\newcommand{\authorB}[2][\@empty]{
\newcommand{\@studentB}{#2}
\newcommand{\@pdfauthorB}{\ifx\@empty#1\relax#2\else#1\fi}
} % end of command \authorB
\newcommand{\ethidB}[1]{\newcommand{\@ethidB}{#1}}
\newcommand{\semesterB}[1]{\newcommand{\@semesterB}{#1}}
\newcommand{\emailB}[1]{\newcommand{\@emailB}{#1}}

% Check if all required document properties are set
\AtBeginDocument{
\if@isgeneric
\@ifundefined{@thetitle}{\ClassError{IDSCreport}{Undefined title}{} \stop}
\@ifundefined{@thedate}{\ClassError{IDSCreport}{Undefined date}{} \stop}
\@ifundefined{@authors}{\ClassError{IDSCreport}{Undefined authors}{} \stop}
\else
\@ifundefined{@thetitle}{\ClassError{IDSCreport}{Undefined title}{} \stop}
\@ifundefined{@thedate}{\ClassError{IDSCreport}{Undefined date}{} \stop}
\@ifundefined{@supervision}{\ClassError{IDSCreport}{Undefined supervision}{} \stop}
\@ifundefined{@identification}{\ClassError{IDSCreport}{Undefined identification number of work}{} \stop}
\@ifundefined{@keywords}{\ClassError{IDSCreport}{Undefined keywords}{} \stop}
\if@isteamwork
\@ifundefined{@studentA}{\ClassError{IDSCreport}{Undefined name of 1st student}{} \stop}
\@ifundefined{@ethidA}{\ClassError{IDSCreport}{Undefined identification number of 1st student}{} \stop}
\@ifundefined{@semesterA}{\ClassError{IDSCreport}{Undefined semester of 1st student}{} \stop}
\@ifundefined{@emailA}{\ClassError{IDSCreport}{Undefined email address of 1st student}{} \stop}
\@ifundefined{@studentB}{\ClassError{IDSCreport}{Undefined name of 2nd student}{} \stop}
\@ifundefined{@ethidB}{\ClassError{IDSCreport}{Undefined identification number of 2nd student}{} \stop}
\@ifundefined{@semesterB}{\ClassError{IDSCreport}{Undefined semester of 2nd student}{} \stop}
\@ifundefined{@emailB}{\ClassError{IDSCreport}{Undefined email address of 2nd student}{} \stop}
\else
\@ifundefined{@student}{\ClassError{IDSCreport}{Undefined name of student}{} \stop}
\@ifundefined{@ethid}{\ClassError{IDSCreport}{Undefined identification number of student}{} \stop}
\@ifundefined{@semester}{\ClassError{IDSCreport}{Undefined semester of student}{} \stop}
\@ifundefined{@email}{\ClassError{IDSCreport}{Undefined email address of student}{} \stop}
\fi % end of isteamwork
\fi % end of isgeneric
} % end \AtBeginDocument



% ==============================================================================
% Environments for document
% ==============================================================================
\NewEnviron{copyrightpage}{\global\let\@copyrightpage\BODY}[\global\@iscopyrightpagetrue]
\RenewEnviron{abstract}{\global\let\@abstract\BODY}[\global\@isabstracttrue]
\NewEnviron{preface}{\global\let\@preface\BODY}[\global\@isprefacetrue]
\NewEnviron{acknowledgment}{\global\let\@acknowledgment\BODY}[\global\@isacknowledgmenttrue]



% ==============================================================================
% Definition of title page
% ==============================================================================
\newcommand{\typesettitlepage}{
\hypersetup{pageanchor=false}
\begin{titlepage}
\pagestyle{empty}
\newgeometry{hmargin=15mm,vmargin=15mm}
\begin{center}

% Institute and ETH logos
\vspace*{-15mm}
\ifpdf % pdflatex
\includegraphics[height=21.5mm, trim=11mm 0 0 0]{docstyle/logos/eth.pdf}
\else % latex (eps-dvi-pdf)
\includegraphics[height=21.5mm, trim=11mm 0 0 0]{docstyle/logos/eth.eps}
\fi
\hfill
\raisebox{7mm}{\setfontsize{11pt}
\if@isgerman
\textsf{Institut f\"{u}r Dynamische Systeme und Regelungstechnik}
\else
\textsf{Institute for Dynamic Systems and Control}
\fi
}

% Author name(s)
\vspace*{3cm}
\if@isgeneric
{\large\@authors}
\vspace{2.5cm}
\else
{\large \if@isteamwork \@studentA{} \& \@studentB \else \@student \fi}
\vspace{1.5cm}
\fi

% Title of work
\begin{minipage}{15cm}
\centering
\bfseries\setfontsize{25pt}{\@thetitle} 
\if@issubtitle \\[0.7cm]\setfontsize{20pt}{\@subtitle}\fi
\end{minipage}
\if@isgeneric
\vspace{4cm}
\else
\vspace{2.5cm}
\fi

% Affiliation
\@ifundefined{@type}{}{\large \textbf{\@type} \\[3ex]}
\if@isgerman
Institut f\"{u}r Dynamische Systeme und Regelungstechnik\\[2mm]
ETH Z\"{u}rich\\
\else
Institute for Dynamic Systems and Control\\[2mm]
ETH Zurich\\
\fi

% Supervisors
\if@isgeneric\else
\vspace{3.5cm}
\if@isgerman
\textbf{Betreuung} \\[1.5ex]
\else
\textbf{Supervision} \\[1.5ex]
\fi % end of isgerman
\@supervision
\fi % end of isgeneric

% Date
\vfill
\@thedate
\vspace*{5mm}

% Report identification number
\if@isgeneric\else
\texttt{\@identification}
\fi % end of isgeneric
\hfill
\raisebox{-7.5mm}{
\ifpdf % pdflatex
\includegraphics[height=15mm]{docstyle/logos/idsc.pdf}
\else % latex (eps-dvi-pdf)
\includegraphics[height=15mm]{docstyle/logos/idsc.eps}
\fi
}

% End of title page
\end{center}
\restoregeometry
\if@iscopyrightpage
\clearpage
~\vfill
\@copyrightpage
\vspace{2cm}
\fi
\@pagebreak % required. Otherwise Abstract appears directly on second page
\end{titlepage}
\hypersetup{pageanchor=true}
} % end command definition of \typesettitlepage



% ==============================================================================
% Definition of the abstract
% ==============================================================================
\newcommand{\typesetabstract}{
\if@isgerman
\originalchapter*{Zusammenfassung}
\else
\originalchapter*{Abstract}
\fi
\@abstract
~\\[1cm]
\@ifundefined{@keywords}{}{
\if@isgerman \textbf{Schlagw\"{o}rter:\quad} \else \textbf{Keywords:\quad}\fi \@keywords.
} % end ifundefined keywords
} % end command definition of \typesetabstract

\newcommand{\typesetabstractwarning}{
\if@isgerman
\originalchapter*{Zusammenfassung}
{\bfseries \color{red}{Zusammenfassung fehlt.}}
\else
\originalchapter*{Abstract}
{\bfseries \color{red}{Abstract is missing.}}
\fi
~\\[1cm]
\textbf{Keywords:} \@keywords.
} % end command definition of \typesetabstractwarning



% ==============================================================================
% Definition of the preface and acknowledgment
% ==============================================================================
\newcommand{\typesetpreface}{
\if@isgerman
\originalchapter*{Vorwort}
\addcontentsline{toc}{chapter}{Vorwort}
\else
\originalchapter*{Preface}
\addcontentsline{toc}{chapter}{Preface}
\fi
\@preface
} % end command definition of \typesetpreface

\newcommand{\typesetacknowledgment}{
\if@isgerman
\originalchapter*{Danksagung}
\else
\originalchapter*{Acknowledgment}
\fi
\@acknowledgment
} % end command definition of \typesetacknowledgment



% ==============================================================================
% Definition of the nomenclature
% ==============================================================================
\newcommand{\typesetnomenclature}{
\if@isgerman
\originalchapter*{Nomenklatur}
\addcontentsline{toc}{chapter}{Nomenklatur}
\else
\originalchapter*{Nomenclature}
\addcontentsline{toc}{chapter}{Nomenclature}
\fi

% Print the acronyms nomenclature
\printnoidxglossary[
type=\acronymtype, % list all glossary entries of type acronym
sort=word, % sort alphabetically
style=acronymtable, % use the style defined above
numberedsection=false, % the title does not have a number
]

% Print the acronyms nomenclature
\printnoidxglossary[
type=symbols, % list all glossary entries of type symbols
sort=word, % sort alphabetically
style=symboltable, % use the style defined above
numberedsection=false, % the title does not have a number
]
} % end command definition of \typesetnomenclature



% ==============================================================================
% Definition of info page (last page)
% ==============================================================================
\newcommand{\typesetinfopage}{
\pagestyle{empty}
\@pagebreak
\newgeometry{noheadfoot,textwidth=160mm,hcentering,vmargin=3cm}

% Include ETH-logo heads of institute 
\ifpdf % pdflatex
\includegraphics[height=21.5mm, trim=7.7mm 0 0 0]{docstyle/logos/eth.pdf}
\else % latex (eps-dvi-pdf)
\includegraphics[height=21.5mm, trim=7.7mm 0 0 0]{docstyle/logos/eth.eps}
\fi
\\[5mm]
\if@isgerman
\textsf{Institut f\"{u}r Dynamische Systeme und Regelungstechnik} \\[5pt]
\else
\textsf{Institute for Dynamic Systems and Control} \\[5pt]
\fi
\textsf{\mbox{Prof. Dr. R. D'Andrea}, \mbox{Prof. Dr. E. Frazzoli}, \mbox{Prof. Dr. Lino Guzzella}, \mbox{Prof. Dr. C. Onder}, \mbox{Prof. Dr. M. Zeilinger}}
\vspace{1.5cm}

% Title of work
\if@isgerman
\textbf{Titel der Arbeit:} \\[1ex]
\else
\textbf{Title of work:} \\[1ex]
\fi
{\LARGE \@thetitletext {\if@issubtitle \par \vspace{-0.3em} \Large \@subtitletext \fi} \par}
\vspace{1cm}

% Type and date
\if@isgerman
\textbf{Art der Arbeit und Datum:} \\[1ex]
\@type, \@thedate\\[2.5ex]
\textbf{Betreuung:} \\[1ex]
\@supervision
\else
\textbf{Thesis type and date:} \\[1ex]
\@type, \@thedate\\[2.5ex]
\textbf{Supervision:} \\[1ex]
\@supervision
\fi
\vspace{1cm}

% Student identification
\if@isgerman
\textbf{\if@isteamwork Studenten:\else Student: \fi}
\else
\textbf{\if@isteamwork Students:\else Student: \fi}
\fi
\\[1.5ex]
\if@isteamwork
\begin{tabular}{@{}p{2cm}lp{1cm}p{2cm}l}
Name: & \@studentA && Name: & \@studentB \\
E-mail: & \@emailA && E-mail: & \@emailB \\
Legi-Nr.: & \@ethidA && Legi-Nr.: & \@ethidB \\
Semester: & \@semesterA && Semester: & \@semesterB
\end{tabular}
\else
\begin{tabular}{@{}p{2cm}l}
Name: & \@student \\
E-mail: & \@email \\
Legi-Nr.: & \@ethid \\
Semester: & \@semester
\end{tabular}
\fi
\vspace{1cm}

% Declaration of originality
\if@isdeclaration
\if@isgerman
\textbf{Erkl\"{a}rung betreffend Plagiaten:} \\[1ex]
\if@isteamwork
Wir erkl\"{a}ren hiermit mit unserer Unterschrift, das Merkblatt Eigenst\"{a}ndigkeitserkl\"{a}rung zur Kenntnis genommen, die vorliegende Arbeit selbst\"{a}ndig verfasst und die im betroffenen Fachgebiet \"{u}blichen Zitiervorschriften eingehalten zu haben.\\[3ex]
\else
Ich erkl\"{a}re mit meiner Unterschrift, das Merkblatt Eigenst\"{a}ndigkeitserkl\"{a}rung zur Kenntnis genommen, die vorliegende Arbeit selbst\"{a}ndig verfasst und die im betroffenen Fachgebiet
\"{u}blichen Zitiervorschriften eingehalten zu haben.\\[3ex]
\fi
Merkblatt Eigenst\"{a}ndigkeitserkl\"{a}rung:\\[1.5ex]
\begin{minipage}{\textwidth}
\url{https://www.ethz.ch/content/dam/ethz/main/education/rechtliches-abschluesse/leistungskontrollen/plagiat-eigenstaendigkeitserklaerung.pdf}
\end{minipage}
\\[6ex] 
Z\"{u}rich, \the\day. \the\month. \the\year: \hrulefill
\else % in english
\textbf{Statement regarding plagiarism:} \\[1ex]
\if@isteamwork % more than one student
By signing this statement, we affirm that we have read and signed the Declaration of Originality, independently produced this paper, and adhered to the general practice of source citation in this subject-area.\\[3ex]
\else % only one student
By signing this statement, I affirm that I have read and signed the Declaration of Originality,
independently produced this paper, and adhered to the general practice of source
citation in this subject-area.\\[3ex]
\fi % end 
Declaration of Originality:\\[1.5ex]
\begin{minipage}{\textwidth}
\RaggedRight
\url{https://www.ethz.ch/content/dam/ethz/main/education/rechtliches-abschluesse/leistungskontrollen/declaration-originality.pdf}
\end{minipage}
\\[6ex] 
Zurich, \the\day.\the\month.\the\year: \hrulefill
\fi % end @isgerman
\fi % end @isdeclaration

% End of infopage
\restoregeometry
} % end command definition of \typesetinfopage