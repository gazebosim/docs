# Maintainer Guidelines

This document provides guidelines and tools for Gazebo maintainers.

## Gazebo Release name selection

Each year, while preparing to make a new Gazebo release, the PMC selects a new
name for the following year's release. The release name selection process
typically starts four weeks before the upcoming (already named) Gazebo release.
This is so that the new name can be announced in the community meeting which
will be held for the release demo. The new name will also be included in the
Discourse post announcing the release. The following is a schedule of tasks that
need to be carried out by the person who will be running the naming process.

- 4 weeks before release, 3 weeks before community meeting: Start a new column
  in the
  [Gazebo release names spreadsheet](https://docs.google.com/spreadsheets/d/1dveyz4Oc3akVfmwRcgmYxy9KUuy7b5aRJx6kUgb9gq0/edit?pli=1&gid=0#gid=0) (not public)
  and invite project committers to start populating the column with
  potential names for the new release. Remind everyone about the naming theme
  and add a couple of names to the column as examples.
- 3 weeks before release, 2 weeks before community meeting:
  - During the week's PMC meeting, filter the names taking into account the
    meaning, trademarks, ease of spelling, ease of searching, etc.
  - Assuming the filtering is completed, create a poll on Discourse asking the
    community to vote for their favorite name from the filtered list. Explain
    that the voting will use a ranked choice system (see
    [example from Jetty](https://discourse.openrobotics.org/t/name-the-next-gazebo-release-gazebo-k/50040)).
    The voting should close a day before the community meeting that will be held
    to present demos from the new release.
- 2 weeks before the release, 1 week before community meeting:
  - Remind the community to vote
- 2 days before community meeting:
  - Send last reminder to vote
- 1 day before community meeting:
  - Record the final name selected by the community.

### Name selection timelines from previous releases

- Jetty -> Gazebo-K timeline
  - Sept 3 - Asked committers to suggest names
  - Sept 8 - Discussed during PMC meeting
  - Sept 12 - [Post on discourse asking the community to vote](https://discourse.openrobotics.org/t/name-the-next-gazebo-release-gazebo-k/50040)
  - Sept 29 - Voting closed
    - In most years, the community meeting (release demos) comes before the
      release. For Jetty, we did it after, so the new name was announced in the
      Discourse post first.
  - Sep 30 - [Release](https://discourse.openrobotics.org/t/gazebo-jetty-released/50349)
  - Oct 1 - [Community meeting (Demo)](https://vimeo.com/1123639071)
- Ionic -> Gazebo-J timeline
  - Aug 26 - Asked committers to suggest names
  - Sep 9 - Discussed during PMC meeting
  - Sep 9 - [Post on discourse asking the community to vote](https://discourse.openrobotics.org/t/name-the-next-gazebo-release-gazebo-j/48589)
  - Sep 25 - Voting closed
  - Sep 25 - [Community meeting (Demo)](https://vimeo.com/1014479065), selected name was announced.
  - Sep 30 - [Release](https://discourse.openrobotics.org/t/gazebo-ionic-release/49064)

## Backporting

When a pull request is merged into the `main` development branch or a stable branch, it may be necessary to backport or forward-port it to other supported (non-EOL) collections if it fixes a bug or adds a non-breaking feature. Even when a PR is merged into an older stable branch, we still use the `@mergifyio backport` command to propagate it forward to newer branches including `main`.

```{important}
When merging a PR that will be backported or forward-ported, you **must use "Rebase and merge"**. This ensures a cleaner git history and properly maintains the original author's commit attribution across branches.
```

```{important}
In the event you need to fix conflicts in a backport, please ask another committer to review your pull request.
```

To port a merged PR to other branches, we use Mergify. You can comment on the PR with the `@mergifyio backport <branches...>` command. The tool below helps you quickly generate the correct Mergify command for any given PR by fetching the PR's target branch and looking up all other supported collection branches in [`gz-collections.yaml`](https://github.com/gazebo-tooling/release-tools/blob/master/jenkins-scripts/dsl/gz-collections.yaml).

```{raw} html
<style>
  /* Custom placeholder styling */
  #pr-url::placeholder {
    color: var(--pst-color-text-muted);
    opacity: 0.4;
  }
</style>
<div id="backport-tool-container" class="card p-4 mt-4" style="background-color: var(--pst-color-surface); color: var(--pst-color-text-base); border-color: var(--pst-color-border);">
  <h4 class="mb-3 mt-0">Generate Mergify Backport Command</h4>
  <label for="pr-url" class="form-label fw-bold">GitHub PR URL:</label>
  <input type="text" id="pr-url" placeholder="https://github.com/gazebosim/gz-sim/pull/123" class="form-control mb-3" style="background-color: var(--pst-color-background); color: var(--pst-color-text-base); border-color: var(--pst-color-border);" />
  <div>
    <button id="generate-btn" class="btn btn-primary btn-sm fw-bold">Generate Command</button>
  </div>
  
  <div class="mt-4">
    <label for="output-cmd" class="form-label fw-bold">Mergify Command:</label>
    <textarea id="output-cmd" readonly class="form-control font-monospace" style="height: 80px; resize: vertical; background-color: var(--pst-color-background); color: var(--pst-color-text-base); border-color: var(--pst-color-border);"></textarea>
    <div class="d-flex align-items-center mt-3">
      <button id="copy-btn" class="btn btn-outline-secondary btn-sm fw-bold">Copy to Clipboard</button>
      <span id="status-msg" class="ms-3 fw-bold small"></span>
    </div>
  </div>
</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/js-yaml/4.1.0/js-yaml.min.js"></script>
<script>
document.addEventListener('DOMContentLoaded', () => {
  const generateBtn = document.getElementById('generate-btn');
  const copyBtn = document.getElementById('copy-btn');
  const prUrlInput = document.getElementById('pr-url');
  const outputCmd = document.getElementById('output-cmd');
  const statusMsg = document.getElementById('status-msg');

  generateBtn.addEventListener('click', async () => {
    statusMsg.textContent = 'Generating...';
    statusMsg.className = 'ms-3 fw-bold text-primary';
    outputCmd.value = '';
    
    const urlStr = prUrlInput.value.trim();
    if (!urlStr) {
      statusMsg.textContent = 'Please enter a PR URL.';
      statusMsg.className = 'ms-3 fw-bold text-danger';
      return;
    }

    const match = urlStr.match(/github\.com\/([^\/]+)\/([^\/]+)\/pull\/(\d+)/);
    if (!match) {
      statusMsg.textContent = 'Invalid PR URL format.';
      statusMsg.className = 'ms-3 fw-bold text-danger';
      return;
    }

    const owner = match[1];
    const repo = match[2];
    const prNumber = match[3];

    try {
      const prRes = await fetch(`https://api.github.com/repos/${owner}/${repo}/pulls/${prNumber}`);
      if (!prRes.ok) {
        if (prRes.status === 403) throw new Error('GitHub API Rate Limit exceeded.');
        if (prRes.status === 404) throw new Error('PR not found.');
        throw new Error(`GitHub API error: ${prRes.status}`);
      }
      const prData = await prRes.json();
      const targetBranch = prData.base.ref;

      const yamlRes = await fetch('https://raw.githubusercontent.com/gazebo-tooling/release-tools/master/jenkins-scripts/dsl/gz-collections.yaml');
      if (!yamlRes.ok) throw new Error(`Failed to fetch collections: ${yamlRes.status}`);
      const yamlText = await yamlRes.text();
      
      const data = jsyaml.load(yamlText);
      
      const branches = new Set(['main']);
      let repoFound = false;

      if (data && data.collections) {
        data.collections.forEach(collection => {
          if (collection.libs) {
            collection.libs.forEach(lib => {
              if (lib.name === repo) {
                repoFound = true;
                if (lib.repo && lib.repo.current_branch) {
                  branches.add(lib.repo.current_branch);
                }
              }
            });
          }
        });
      }

      if (!repoFound) {
        statusMsg.textContent = `Repository '${repo}' not found in collections.`;
        statusMsg.className = 'ms-3 fw-bold text-danger';
        return;
      }

      branches.delete(targetBranch);

      if (branches.size === 0) {
        statusMsg.textContent = 'No backport branches available.';
        statusMsg.className = 'ms-3 fw-bold text-success';
        return;
      }

      const sortedBranches = Array.from(branches).sort();
      const command = `@mergifyio backport ${sortedBranches.join(' ')}`;
      
      outputCmd.value = command;
      statusMsg.textContent = 'Success!';
      statusMsg.className = 'ms-3 fw-bold text-success';

    } catch (err) {
      statusMsg.textContent = err.message;
      statusMsg.className = 'ms-3 fw-bold text-danger';
      console.error(err);
    }
  });

  copyBtn.addEventListener('click', () => {
    if (outputCmd.value) {
      outputCmd.select();
      document.execCommand('copy');
      statusMsg.textContent = 'Copied to clipboard!';
      statusMsg.className = 'ms-3 fw-bold text-success';
      setTimeout(() => {
        if (statusMsg.textContent === 'Copied to clipboard!') {
           statusMsg.textContent = 'Success!';
        }
      }, 2000);
    }
  });
});
</script>
```
