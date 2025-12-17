---
sort: 4
---

## Customizing Bash in Linux (OPTIONAL)

Customizing your Bash environment can improve your productivity and enhance the aesthetics of your terminal. In this guide, we'll discuss how to change the Bash theme to Dracula and use Starship for a modern and informative prompt.

### 1. Changing the Bash Theme to Dracula

The Dracula theme is a dark theme popular among developers. Here's how to set it up:

1. **Terminal Theme**: 
   - Most modern terminals support the Dracula theme. Visit the [Dracula Official Site](https://draculatheme.com/) to find specific instructions for your terminal emulator.

2. **Bash Syntax Highlighting**:
   - Clone the Bash syntax-highlighting repo:
     \```bash
     git clone https://github.com/zdharma/fast-syntax-highlighting.git ~/.fast-syntax-highlighting
     \```
   - Add the following to your `~/.bashrc`:
     \```bash
     source ~/.fast-syntax-highlighting/fast-syntax-highlighting.plugin.zsh
     \```

```note
Bonus Tip. Dracula theme can be applied to many different programs. Try it with [VSCode}(https://draculatheme.com/visual-studio-code)
```

### 2. Using Starship for Prompt Customization

[Starship](https://starship.rs/) is a cross-shell prompt that's minimal, fast, and infinitely customizable. 

1. **Installation**:
   - Install Starship using a package manager or via a script:
     \```bash
     curl -fsSL https://starship.rs/install.sh | bash
     \```

2. **Configuration**:
   - By default, Starship will use its default configuration. To customize it, create a configuration file:
     \```bash
     touch ~/.config/starship.toml
     \```
   - Edit this file to customize the prompt. Refer to the [Starship Configuration Documentation](https://starship.rs/config/) for details.

3. **Integrate with Bash**:
   - Add the following to your `~/.bashrc`:
     \```bash
     eval "$(starship init bash)"
     \```

### Tips

- Always back up your configuration files (like `.bashrc`) before making significant changes.
- Explore the Starship and Dracula documentation for advanced configurations and tweaks.
- Remember to `source ~/.bashrc` or open a new terminal to see the changes take effect.
