# project-polaris


<table style="border: none; border-collapse: collapse;">
 	<tr>
        <td valign="middle" style="border: none; padding-left: 12px;">
 			<p style="margin:4px 0 0 0"><strong><em>POLARIS Focus Project 2026</em></strong><br/>
 			Our goal is to develop and deploy an autonomous underwater vehicle that is capable of navigating in frozen lakes, with freezing conditions and take ice-thickness measurements from beneath the ice.</p>
 		</td>
 		<td width="140" valign="middle" style="border: none; padding: 0;">
 			<img src="docs/logo.png" alt="POLARIS logo" width="120" style="border:none; display:block;" />
 		</td>
 	</tr>
</table>

### Getting Started

**0. Prerequisites**
 This repo is made for ROS2-humble, which runs on Ubuntu 22.04.

**1. Clone Repository**
 Work in progress...

**2.a Run on local machine**
 Work in progress...

**2.b Run on Docker container**
 Work in progress...

**3. Build ROS2 Packages**
 Work in progress...

---
### Usage
Work in progress...

---

### Repository Structure
**.github**
Here your git-specific files are stored that are needed for example to automate workflows.

**config**
Contains configuration files to collect essential variables that can be adjusted here.

**scripts**
Drop off your random scripts that are not essential for operation but might come in handy some time.

**src**
All the main code and ROS2 packages belong here. You can add prototype packages but name them with "pt_..." to mark it as prototype and add it to colconignore.

**tests**
Here would belong git-standardized test protocols used in CI, for now it should stay empty since no CI is planned.

**.colconignore**
Add ROS2-packages into this list that should not be built automatically, for example demo_package.

**.gitignore**
Everything that is not needed in the main repo should be put on this list. This is done to keep a clean main repo. Examples for files that should be put in this list are: Any kind of log files, pycache folders,...

---

### Workflow for implementing a new feature:

**Setup:**
Always begin by going to the develop branch.
Create a separate branch for the new feature to keep changes isolated.


**Developing the Feature:**
Implement the feature and test it thoroughly.
Make regular commits with clear descriptions of changes.
Make sure that everything is properly commented, so that the reviewers and future you understand the code
Document non-code related information in Notion, e.g. hardware setup guides, wiring charts, configuration steps etc.
Push the branch to the remote repository to ensure changes are tracked.


**Opening a Pull Request:**
Once the feature is working, create a pull request to merge the feature branch into develop.
At least two team members must review and approve the pull request.
Address any feedback and make necessary improvements.


**Merging into Develop:**
After approval, merge the feature branch into develop.


**Merging into Main for Integration:**
When the develop branch reaches a stable and fully integrated state, it is merged into the main branch.
If applicable, tag the new version and prepare for deployment.
Update any relevant documentation.


**Branching Strategy:**

main:          stable, production-ready

develop:       ongoing development

feature/name:  one branch per feature or fix
