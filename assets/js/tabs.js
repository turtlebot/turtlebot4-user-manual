const removeActiveClasses = function (ulElement) {
    const lis = ulElement.querySelectorAll('li');
    Array.prototype.forEach.call(lis, function(li) {
        li.classList.remove('active');
    });
}

const addActiveClasses = function (ulElement) {
    const lis = ulElement.querySelectorAll('li');
    Array.prototype.forEach.call(lis, function(li) {
        li.classList.add('active');
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
    // List item representing the tab
    liTab = link.parentNode;
    // Unordered list that holds the tabs
    ulTab = liTab.parentNode;
    // Position of active tab
    position = getChildPosition(liTab);

    // Make tab active
    addActiveClasses(liTab);
    liTab.classList.add('active');

    // Get tab contents
    tabContentId = ulTab.getAttribute('data-tab');
    tabContentElement = document.getElementById(tabContentId);

    // Make tab contents active
    tabContent = tabContentElement.querySelectorAll('.tab-content > li');
    activeTabContent = tabContent[position];
    addActiveClasses(activeTabContent)
    activeTabContent.classList.add('active');
    
}

const removeActiveClass = function(link) {
    // List item representing the tab
    liTab = link.parentNode;
    // Unordered list that holds the tabs
    ulTab = liTab.parentNode;
    // Position of active tab
    position = getChildPosition(liTab);

    // Remove tab active
    removeActiveClasses(liTab);
    liTab.classList.remove('active');

    // Get tab contents
    tabContentId = ulTab.getAttribute('data-tab');
    tabContentElement = document.getElementById(tabContentId);

    // Make tab contents inactive
    tabContent = tabContentElement.querySelectorAll('.tab-content > li');
    inactiveTabContent = tabContent[position];
    removeActiveClasses(inactiveTabContent);
    inactiveTabContent.classList.remove('active');
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
        else
        {
            removeActiveClass(link);
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
            else
            {
                removeActiveClass(link);
            }
        });
      }, false);
    });
});