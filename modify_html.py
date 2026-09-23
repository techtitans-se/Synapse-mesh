import re

with open('/home/akil/Desktop/synapse_mesh/src/amr_web_dashboard/index.html', 'r') as f:
    html = f.read()

# Define the new tab structure wrapping the existing content

telemetry_start = html.find('<!-- Telemetry -->')
stlease_start = html.find('<!-- ST-Lease Fleet Manager -->')
vos_start = html.find('<!-- VOS Live Monitor Panel -->')
side_panel_end = html.find('</section>', vos_start)

# Split the content
general_content = html[telemetry_start:stlease_start]
stlease_content = html[stlease_start:vos_start]
vos_content = html[vos_start:side_panel_end]

tabs_header = """
                <!-- Tabs Header -->
                <div class="tabs-header" style="display: flex; gap: 12px; margin-bottom: 8px;">
                    <button class="tab-btn active" id="btn-tab-general" onclick="switchTab('tab-general')">Dashboard</button>
                    <button class="tab-btn" id="btn-tab-stlease" onclick="switchTab('tab-stlease')">ST-Lease</button>
                    <button class="tab-btn" id="btn-tab-vos" onclick="switchTab('tab-vos')">VOS Monitor</button>
                </div>

                <!-- Tab Content: General -->
                <div id="tab-general" class="tab-content active" style="display: flex; flex-direction: column; gap: 24px;">
"""

stlease_tab = """
                </div>
                <!-- Tab Content: ST-Lease -->
                <div id="tab-stlease" class="tab-content" style="display: none; flex-direction: column; gap: 24px;">
"""

vos_tab = """
                </div>
                <!-- Tab Content: VOS Monitor -->
                <div id="tab-vos" class="tab-content" style="display: none; flex-direction: column; gap: 24px;">
"""

end_tabs = "\n                </div>\n"

new_side_panel = tabs_header + general_content + stlease_tab + stlease_content + vos_tab + vos_content + end_tabs

# Reconstruct
new_html = html[:telemetry_start] + new_side_panel + html[side_panel_end:]

with open('/home/akil/Desktop/synapse_mesh/src/amr_web_dashboard/index.html', 'w') as f:
    f.write(new_html)

print("HTML structure updated successfully.")
