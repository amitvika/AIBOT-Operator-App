import webview
import os
import sys
from datetime import datetime
from firestore_db import FirestoreDB
from login_register import LoginRegisterModule

# Import tab modules
from operator_tasks import get_operator_tasks_html
from supervise import get_supervise_html
from profile import get_profile_html

# Global variables
db = None
current_user = None
window = None
login_module = None
initial_html_loaded = False  # Flag to prevent infinite loading

def logout():
    """Handle user logout"""
    global current_user, db, window, login_module
    if current_user:
        db.update_operator_status(current_user['email'], False)
        current_user = None
    window.load_html(login_module.get_login_html())
    return True

def on_close():
    """Handle application close"""
    global current_user, db
    if current_user:
        db.update_operator_status(current_user['email'], False)
    return True

def load_tab(tab_name):
    """Return HTML content for the selected tab"""
    global current_user
    if tab_name == "Operator Tasks":
        return get_operator_tasks_html(db)
    elif tab_name == "Supervise":
        return get_supervise_html()
    elif tab_name == "Profile":
        return get_profile_html(current_user)
    else:
        return "<div><p>Unknown tab selected</p></div>"

def show_main_app(user_data):
    """Show the main application after successful login with tabs."""
    global window, current_user, db, login_module

    # Update current user and their status
    current_user = user_data
    db.update_operator_status(user_data['email'], True)

    # Retrieve the logo in base64 format using the login module
    logo_base64 = login_module.get_logo_base64()
    logo_img_html = (
        f'<img src="data:image/png;base64,{logo_base64}" alt="Logo" class="navbar-logo">'
        if logo_base64 else '<span>Logo</span>'
    )

    # Default content for the first tab (Operator Tasks)
    default_tab_html = get_operator_tasks_html(db)

    main_app_html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Main Application</title>
        <style>
            body {{
                font-family: Arial, sans-serif;
                margin: 0;
                padding: 0;
                background-color: #f5f5f5;
            }}
            /* Navbar styles */
            .navbar {{
                display: flex;
                justify-content: space-between;
                align-items: center;
                background-color: #333;
                padding: 10px 20px;
                color: white;
            }}
            .navbar-logo {{
                height: 40px;
            }}
            .navbar-tabs {{
                display: flex;
                gap: 15px;
            }}
            .tab-btn {{
                background: none;
                border: none;
                color: white;
                font-size: 16px;
                cursor: pointer;
                padding: 8px 12px;
            }}
            .tab-btn:hover {{
                background-color: #444;
            }}
            .logout-btn {{
                background-color: #f44336;
                border: none;
                color: white;
                padding: 8px 16px;
                cursor: pointer;
                border-radius: 4px;
                font-size: 16px;
            }}
            .tab-content {{
                padding: 20px;
                background-color: white;
                margin: 20px;
                border-radius: 5px;
                box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            }}
        </style>
    </head>
    <body>
        <nav class="navbar">
            <div class="navbar-left">
                {logo_img_html}
            </div>
            <div class="navbar-tabs">
                <button class="tab-btn" onclick="switchTab('Operator Tasks')">Operator Tasks</button>
                <button class="tab-btn" onclick="switchTab('Supervise')">Supervise</button>
                <button class="tab-btn" onclick="switchTab('Profile')">Profile</button>
            </div>
            <div class="navbar-right">
                <button class="logout-btn" onclick="window.pywebview.api.logout()">Logout</button>
            </div>
        </nav>
        <div id="tab-content" class="tab-content">
            {default_tab_html}
        </div>
        <script>
            function switchTab(tabName) {{
                window.pywebview.api.load_tab(tabName).then(function(htmlContent) {{
                    document.getElementById('tab-content').innerHTML = htmlContent;
                }});
            }}
            window.addEventListener('beforeunload', function() {{
                window.pywebview.api.on_close();
            }});
        </script>
    </body>
    </html>
    """
    window.load_html(main_app_html)

def on_loaded():
    global initial_html_loaded, login_module, window
    if not initial_html_loaded:
        initial_html_loaded = True
        window.load_html(login_module.get_login_html())

def main():
    global db, window, login_module, initial_html_loaded

    # Initialize database
    db = FirestoreDB('creds.json')

    # Create window with initial empty HTML
    window = webview.create_window(
        'My Application',
        html="<html><body><h1>Loading...</h1></body></html>",
        width=800,
        height=600,
        resizable=True,
        min_size=(600, 400)
    )

    initial_html_loaded = False  # Ensure the flag is reset

    # Initialize login module
    login_module = LoginRegisterModule(db, show_main_app)

    # Expose API functions to JavaScript
    window.expose(login_module.login)
    window.expose(login_module.register)
    window.expose(login_module.toggle_form)
    window.expose(logout)
    window.expose(on_close)
    window.expose(load_tab)

    # Set the loaded event handler to load the login page only once
    window.events.loaded += on_loaded

    # Start webview
    webview.start(debug=True)

if __name__ == '__main__':
    main()
