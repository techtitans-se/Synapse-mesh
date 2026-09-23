with open('/home/akil/Desktop/synapse_mesh/src/amr_web_dashboard/main.js', 'a') as f:
    f.write("""

// Tab Switching Logic
window.switchTab = function(tabId) {
    // Hide all tab contents
    document.querySelectorAll('.tab-content').forEach(el => {
        el.style.display = 'none';
        el.classList.remove('active');
    });

    // Remove active class from all buttons
    document.querySelectorAll('.tab-btn').forEach(el => {
        el.classList.remove('active');
    });

    // Show target tab
    const targetTab = document.getElementById(tabId);
    if (targetTab) {
        targetTab.style.display = 'flex';
        targetTab.classList.add('active');
    }

    // Add active class to corresponding button
    const targetBtn = document.getElementById('btn-' + tabId);
    if (targetBtn) {
        targetBtn.classList.add('active');
    }
};

""")
print("JS updated successfully.")
