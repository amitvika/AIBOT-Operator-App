import os
import base64
from datetime import datetime

class LoginRegisterModule:
    def __init__(self, db, on_success_callback):
        """Initialize the login/register module
        
        Args:
            db: Firestore database instance
            on_success_callback: Function to call after successful login/registration
        """
        self.db = db
        self.on_success = on_success_callback
    
    def get_logo_base64(self):
        """Get the logo as base64 encoded string or return empty if not found"""
        logo_path = os.path.join('assets', 'logo.png')
        if os.path.exists(logo_path):
            with open(logo_path, 'rb') as image_file:
                return base64.b64encode(image_file.read()).decode('utf-8')
        return ''
    
    def get_login_html(self):
        """Generate the HTML for login/register page"""
        logo_base64 = self.get_logo_base64()
        logo_html = f'<img src="data:image/png;base64,{logo_base64}" alt="Logo" class="logo">' if logo_base64 else ''
        
        return f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Login/Register</title>
            <style>
                body {{
                    font-family: Arial, sans-serif;
                    background-color: #f5f5f5;
                    display: flex;
                    justify-content: center;
                    align-items: center;
                    height: 100vh;
                    margin: 0;
                }}
                .container {{
                    background-color: white;
                    border-radius: 8px;
                    box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
                    padding: 30px;
                    width: 350px;
                    text-align: center;
                }}
                .logo {{
                    max-width: 150px;
                    margin-bottom: 20px;
                }}
                h2 {{
                    color: #333;
                    margin-bottom: 20px;
                }}
                .input-group {{
                    margin-bottom: 15px;
                    text-align: left;
                }}
                .input-group label {{
                    display: block;
                    margin-bottom: 5px;
                    color: #555;
                }}
                .input-group input {{
                    width: 100%;
                    padding: 10px;
                    border: 1px solid #ddd;
                    border-radius: 4px;
                    box-sizing: border-box;
                }}
                .row {{
                    display: flex;
                    gap: 10px;
                }}
                .row .input-group {{
                    flex: 1;
                }}
                button {{
                    width: 100%;
                    padding: 12px;
                    background-color: #4285f4;
                    color: white;
                    border: none;
                    border-radius: 4px;
                    cursor: pointer;
                    font-size: 16px;
                    margin-top: 10px;
                }}
                button:hover {{
                    background-color: #3367d6;
                }}
                .form-toggle {{
                    margin-top: 20px;
                    color: #4285f4;
                    cursor: pointer;
                }}
                .error-message {{
                    color: #f44336;
                    margin: 10px 0;
                    display: none;
                }}
                .hide {{
                    display: none;
                }}
            </style>
        </head>
        <body>
            <div class="container">
                {logo_html}
                
                <h2 id="form-title">Login</h2>
                <div id="error-message" class="error-message"></div>
                
                <!-- Login Form -->
                <div id="login-form">
                    <div class="input-group">
                        <label for="login-email">Email</label>
                        <input type="email" id="login-email" placeholder="Enter your email">
                    </div>
                    <div class="input-group">
                        <label for="login-password">Password</label>
                        <input type="password" id="login-password" placeholder="Enter your password">
                    </div>
                    <button onclick="login()">Login</button>
                    <div class="form-toggle" onclick="toggleForm()">Don't have an account? Register</div>
                </div>
                
                <!-- Register Form -->
                <div id="register-form" class="hide">
                    <div class="row">
                        <div class="input-group">
                            <label for="first-name">First Name</label>
                            <input type="text" id="first-name" placeholder="First name">
                        </div>
                        <div class="input-group">
                            <label for="last-name">Last Name</label>
                            <input type="text" id="last-name" placeholder="Last name">
                        </div>
                    </div>
                    <div class="input-group">
                        <label for="register-email">Email</label>
                        <input type="email" id="register-email" placeholder="Enter your email">
                    </div>
                    <div class="input-group">
                        <label for="register-password">Password</label>
                        <input type="password" id="register-password" placeholder="Create a password">
                    </div>
                    <div class="input-group">
                        <label for="confirm-password">Confirm Password</label>
                        <input type="password" id="confirm-password" placeholder="Confirm your password">
                    </div>
                    <button onclick="register()">Register</button>
                    <div class="form-toggle" onclick="toggleForm()">Already have an account? Login</div>
                </div>
            </div>
            
            <script>
                function toggleForm() {{
                    document.getElementById('login-form').classList.toggle('hide');
                    document.getElementById('register-form').classList.toggle('hide');
                    
                    const formTitle = document.getElementById('form-title');
                    formTitle.textContent = formTitle.textContent === 'Login' ? 'Register' : 'Login';
                    
                    document.getElementById('error-message').style.display = 'none';
                    
                    // Call the Python toggle_form function
                    window.pywebview.api.toggle_form();
                }}
                
                function showError(message) {{
                    const errorElement = document.getElementById('error-message');
                    errorElement.textContent = message;
                    errorElement.style.display = 'block';
                }}
                
                function login() {{
                    const email = document.getElementById('login-email').value;
                    const password = document.getElementById('login-password').value;
                    
                    if (!email || !password) {{
                        showError('Please enter both email and password');
                        return;
                    }}
                    
                    window.pywebview.api.login(email, password).then(response => {{
                        if (response && response.error) {{
                            showError(response.error);
                        }}
                    }});
                }}
                
                function register() {{
                    const firstName = document.getElementById('first-name').value;
                    const lastName = document.getElementById('last-name').value;
                    const email = document.getElementById('register-email').value;
                    const password = document.getElementById('register-password').value;
                    const confirmPassword = document.getElementById('confirm-password').value;
                    
                    if (!firstName || !lastName || !email || !password || !confirmPassword) {{
                        showError('Please fill in all fields');
                        return;
                    }}
                    
                    if (password !== confirmPassword) {{
                        showError('Passwords do not match');
                        return;
                    }}
                    
                    window.pywebview.api.register(firstName, lastName, email, password).then(response => {{
                        if (response && response.error) {{
                            showError(response.error);
                        }}
                    }});
                }}
            </script>
        </body>
        </html>
        """
    
    def login(self, email, password):
        """Handle login requests from the UI"""
        user = self.db.authenticate_operator(email, password)
        if user:
            self.on_success(user)
            return True
        else:
            return {"error": "Invalid email or password"}
    
    def register(self, first_name, last_name, email, password):
        """Handle registration requests from the UI"""
        # Check if user already exists
        if self.db.check_operator_exists(email):
            return {"error": "User with this email already exists"}
        
        # Create new user
        user_data = {
            "first_name": first_name,
            "last_name": last_name,
            "email": email,
            "password": password,  # In production, you'd want to hash this
            "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "online": True,
            "position": "operator"
        }
        
        result = self.db.create_operator(user_data)
        if result:
            self.on_success(user_data)
            return True
        else:
            return {"error": "Failed to create account"}
            
    def toggle_form(self):
        """Toggle between login and register forms"""
        return True