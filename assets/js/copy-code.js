document.addEventListener("DOMContentLoaded", function () {
    document.querySelectorAll("pre.highlight").forEach((codeBlock) => {
      const button = document.createElement("button");
      button.className = "copy-code-button";
      button.innerText = "Copy";
      codeBlock.appendChild(button);
  
      button.addEventListener("click", function () {
        const code = codeBlock.querySelector("code");
        const text = code.innerText || code.textContent;
        navigator.clipboard.writeText(text).then(
          () => {
            button.innerText = "Copied!";
            setTimeout(() => (button.innerText = "Copy"), 2000);
          },
          (err) => {
            console.error("Copy failed:", err);
          }
        );
      });
    });
  });
  