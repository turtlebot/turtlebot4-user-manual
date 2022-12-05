const removeActiveClasses = function (ulElement) {
    const lis = ulElement.querySelectorAll('li');
    Array.prototype.forEach.call(lis, function(li) {
        li.classList.remove('active');
    });
  }

const getChildPosition = function (element) {
    var parent = element.parentNode;
    var i = 0;
    for (var i = 0; i < parent.children.length; i++) {
        if (parent.children[i] === element) {
            return i;
        }
    }

    throw new Error('No parent found');
}

const setActiveClass = function(link) {
    liTab = link.parentNode;
    ulTab = liTab.parentNode;
    position = getChildPosition(liTab);

    removeActiveClasses(ulTab);
    tabContentId = ulTab.getAttribute('data-tab');
    tabContentElement = document.getElementById(tabContentId);
    removeActiveClasses(tabContentElement);

    // Set active if tab matches rosVersion
    tabContentElement.querySelectorAll('li')[position].classList.add('active');
    liTab.classList.add('active');
}

window.addEventListener('load', function () {
    // Default to Humble
    if (window.localStorage.getItem("rosVersion") === null)
    {
        window.localStorage.setItem("rosVersion", "humble");
    }

    // Get all tab links
    const tabLinks = document.querySelectorAll('ul.tab li a');

    // Set all rosVersion tabs to active
    Array.prototype.forEach.call(tabLinks, function(link) {
        if (link.innerHTML.trim() === window.localStorage.getItem("rosVersion"))
        {
            setActiveClass(link);
        }
    });

    Array.prototype.forEach.call(tabLinks, function(link) {
      link.addEventListener('click', function (event) {
        event.preventDefault();

        // ROS version already in use
        if (link.innerHTML.trim() === window.localStorage.getItem("rosVersion"))
        {
            return;
        }

        window.localStorage.setItem("rosVersion", link.innerHTML.trim());

        Array.prototype.forEach.call(tabLinks, function(link) {
            if (link.innerHTML.trim() === window.localStorage.getItem("rosVersion"))
            {
                setActiveClass(link);
            }
        });
      }, false);
    });
});