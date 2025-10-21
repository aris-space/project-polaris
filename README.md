# project-polaris
POLARIS Focus Project 2026

---

# Workflow for implementing a new feature:

## Setup:
Always begin by going to the develop branch.
Create a separate branch for the new feature to keep changes isolated.


## Developing the Feature:
Implement the feature and test it thoroughly.
Make regular commits with clear descriptions of changes.
Make sure that everything is properly commented, so that the reviewers and future you understand the code
Document non-code related information in Notion, e.g. hardware setup guides, wiring charts, configuration steps etc.
Push the branch to the remote repository to ensure changes are tracked.


## Opening a Pull Request:
Once the feature is working, create a pull request to merge the feature branch into develop.
At least two team members must review and approve the pull request.
Address any feedback and make necessary improvements.


## Merging into Develop:
After approval, merge the feature branch into develop.


## Merging into Main for Integration:
When the develop branch reaches a stable and fully integrated state, it is merged into the main branch.
If applicable, tag the new version and prepare for deployment.
Update any relevant documentation.


Branching Strategy

main             ← stable, production-ready
develop          ← ongoing development
feature/xyz      ← one branch per feature or fix
hotfix/xyz       ← emergency fixes to main
