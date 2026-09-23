with open('/home/akil/Desktop/synapse_mesh/src/amr_web_dashboard/style.css', 'a') as f:
    f.write("""

/* Tabs */
.tabs-header {
    background: rgba(0, 0, 0, 0.2);
    padding: 6px;
    border-radius: 12px;
    border: 1px solid rgba(255, 255, 255, 0.05);
}

.tab-btn {
    flex: 1;
    background: transparent;
    color: var(--text-secondary);
    border: none;
    padding: 10px 16px;
    border-radius: 8px;
    font-size: 0.9rem;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.2s ease;
}

.tab-btn:hover {
    color: var(--text-primary);
    background: rgba(255, 255, 255, 0.05);
}

.tab-btn.active {
    background: linear-gradient(135deg, var(--accent-indigo), var(--accent-blue));
    color: white;
    box-shadow: 0 4px 15px rgba(99, 102, 241, 0.3);
}

.tab-content {
    animation: fadeIn 0.3s ease-in-out;
}

@keyframes fadeIn {
    from { opacity: 0; transform: translateY(5px); }
    to { opacity: 1; transform: translateY(0); }
}

""")
print("CSS updated successfully.")
