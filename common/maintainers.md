# Maintainer Guidelines

This document provides guidelines and tools for Gazebo maintainers.

## Backporting

When a pull request is merged into the `main` development branch or a stable branch, it may be necessary to backport or forward-port it to other supported (non-EOL) collections if it fixes a bug or adds a non-breaking feature. Even when a PR is merged into an older stable branch, we still use the `@mergifyio backport` command to propagate it forward to newer branches including `main`.

```{important}
When merging a PR that will be backported or forward-ported, you **must use "Rebase and merge"**. This ensures a cleaner git history and properly maintains the original author's commit attribution across branches.
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
