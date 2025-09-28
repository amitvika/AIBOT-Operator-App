def get_profile_html(user_data):
    """
    Generate HTML for the Profile tab with editable fields.
    
    Args:
        user_data (dict): Dictionary containing user profile data.
    
    Returns:
        str: HTML string representing the profile form.
    """
    # Get current values with defaults
    first_name = user_data.get('first_name', '')
    last_name = user_data.get('last_name', '')
    email = user_data.get('email', '')
    position = user_data.get('position', '')
    phone = user_data.get('phone', '')
    department = user_data.get('department', '')
    created_at = user_data.get('created_at', 'Unknown')
    last_login = user_data.get('last_login', 'Unknown')
    
    return f"""
    <style>
        .profile-container {{
            max-width: 600px;
            margin: 0 auto;
            padding: 20px;
        }}
        .profile-header {{
            text-align: center;
            margin-bottom: 30px;
            padding-bottom: 20px;
            border-bottom: 2px solid #e9ecef;
        }}
        .profile-header h2 {{
            color: #333;
            margin-bottom: 10px;
        }}
        .profile-form {{
            background-color: #f8f9fa;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }}
        .form-group {{
            margin-bottom: 20px;
        }}
        .form-group label {{
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
            color: #495057;
        }}
        .form-group input,
        .form-group select,
        .form-group textarea {{
            width: 100%;
            padding: 10px;
            border: 1px solid #ced4da;
            border-radius: 5px;
            font-size: 14px;
            box-sizing: border-box;
        }}
        .form-group input:focus,
        .form-group select:focus,
        .form-group textarea:focus {{
            outline: none;
            border-color: #007bff;
            box-shadow: 0 0 0 2px rgba(0,123,255,0.25);
        }}
        .form-row {{
            display: flex;
            gap: 15px;
        }}
        .form-row .form-group {{
            flex: 1;
        }}
        .readonly-field {{
            background-color: #e9ecef;
            color: #6c757d;
            cursor: not-allowed;
        }}
        .button-group {{
            display: flex;
            gap: 10px;
            justify-content: center;
            margin-top: 30px;
        }}
        .btn {{
            padding: 12px 24px;
            border: none;
            border-radius: 5px;
            font-size: 16px;
            cursor: pointer;
            transition: background-color 0.3s;
        }}
        .btn-primary {{
            background-color: #007bff;
            color: white;
        }}
        .btn-primary:hover {{
            background-color: #0056b3;
        }}
        .btn-secondary {{
            background-color: #6c757d;
            color: white;
        }}
        .btn-secondary:hover {{
            background-color: #545b62;
        }}
        .btn-success {{
            background-color: #28a745;
            color: white;
        }}
        .btn-success:hover {{
            background-color: #1e7e34;
        }}
        .alert {{
            padding: 12px;
            margin-bottom: 20px;
            border-radius: 5px;
            display: none;
        }}
        .alert-success {{
            background-color: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }}
        .alert-error {{
            background-color: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }}
        .account-info {{
            background-color: #e9ecef;
            padding: 15px;
            border-radius: 5px;
            margin-top: 20px;
        }}
        .account-info h4 {{
            margin-top: 0;
            color: #495057;
        }}
        .account-info p {{
            margin: 5px 0;
            color: #6c757d;
        }}
    </style>
    <div class="profile-container">
        <div class="profile-header">
            <h2>User Profile</h2>
            <p>Manage your personal information and account settings</p>
        </div>
        
        <div id="alert-container"></div>
        
        <form class="profile-form" id="profileForm">
            <div class="form-row">
                <div class="form-group">
                    <label for="first_name">First Name *</label>
                    <input type="text" id="first_name" name="first_name" value="{first_name}" required>
                </div>
                <div class="form-group">
                    <label for="last_name">Last Name *</label>
                    <input type="text" id="last_name" name="last_name" value="{last_name}" required>
                </div>
            </div>
            
            <div class="form-group">
                <label for="email">Email Address</label>
                <input type="email" id="email" name="email" value="{email}" class="readonly-field" readonly>
                <small style="color: #6c757d;">Email cannot be changed</small>
            </div>
            
            <div class="form-row">
                <div class="form-group">
                    <label for="position">Position/Title</label>
                    <input type="text" id="position" name="position" value="{position}">
                </div>
                <div class="form-group">
                    <label for="department">Department</label>
                    <select id="department" name="department">
                        <option value="">Select Department</option>
                        <option value="Operations" {'selected' if department == 'Operations' else ''}>Operations</option>
                        <option value="Maintenance" {'selected' if department == 'Maintenance' else ''}>Maintenance</option>
                        <option value="Quality Control" {'selected' if department == 'Quality Control' else ''}>Quality Control</option>
                        <option value="Engineering" {'selected' if department == 'Engineering' else ''}>Engineering</option>
                        <option value="Supervision" {'selected' if department == 'Supervision' else ''}>Supervision</option>
                        <option value="Administration" {'selected' if department == 'Administration' else ''}>Administration</option>
                    </select>
                </div>
            </div>
            
            <div class="form-group">
                <label for="phone">Phone Number</label>
                <input type="tel" id="phone" name="phone" value="{phone}" placeholder="(555) 123-4567">
            </div>
            
            <div class="button-group">
                <button type="button" class="btn btn-primary" onclick="saveProfile()">Save Changes</button>
                <button type="button" class="btn btn-secondary" onclick="resetForm()">Reset</button>
            </div>
        </form>
        
        <div class="account-info">
            <h4>Account Information</h4>
            <p><strong>Account Created:</strong> {created_at}</p>
            <p><strong>Last Login:</strong> {last_login}</p>
            <p><strong>User ID:</strong> {user_data.get('id', 'N/A')}</p>
        </div>
    </div>
    
    <script>
        let originalData = {{
            first_name: '{first_name}',
            last_name: '{last_name}',
            position: '{position}',
            department: '{department}',
            phone: '{phone}'
        }};
        
        function showAlert(message, type) {{
            const alertContainer = document.getElementById('alert-container');
            alertContainer.innerHTML = `
                <div class="alert alert-${{type}}">
                    ${{message}}
    </div>
            `;
            const alert = alertContainer.querySelector('.alert');
            alert.style.display = 'block';
            
            // Auto-hide after 5 seconds
            setTimeout(() => {{
                alert.style.display = 'none';
            }}, 5000);
        }}
        
        function saveProfile() {{
            console.log('DEBUG: saveProfile function called');
            
            const form = document.getElementById('profileForm');
            const formData = new FormData(form);
            const profileData = Object.fromEntries(formData.entries());
            
            console.log('DEBUG: Profile data collected:', profileData);
            console.log('DEBUG: API available:', typeof window.pywebview.api.update_profile);
            
            // Validate required fields
            if (!profileData.first_name.trim() || !profileData.last_name.trim()) {{
                showAlert('First name and last name are required.', 'error');
                return;
            }}
            
            // Validate phone number format (optional)
            if (profileData.phone && !/^[\d\s\-\(\)\+]+$/.test(profileData.phone)) {{
                showAlert('Please enter a valid phone number.', 'error');
                return;
            }}
            
            console.log('DEBUG: Calling update_profile API...');
            
            // Call the Python backend to save the profile
            window.pywebview.api.update_profile(profileData).then(function(success) {{
                console.log('DEBUG: API response received:', success);
                if (success) {{
                    showAlert('Profile updated successfully!', 'success');
                    // Update original data to reflect changes
                    originalData = {{...profileData}};
                    
                    // Refresh user data from database to get the latest changes
                    window.pywebview.api.refresh_user_data().then(function(refreshSuccess) {{
                        console.log('DEBUG: Refresh response:', refreshSuccess);
                        if (refreshSuccess) {{
                            // Reload the profile tab to show updated data
                            window.pywebview.api.load_tab('Profile').then(function(htmlContent) {{
                                document.getElementById('tab-content').innerHTML = htmlContent;
                            }});
                        }}
                    }});
                }} else {{
                    showAlert('Failed to update profile. Please try again.', 'error');
                }}
            }}).catch(function(error) {{
                console.log('DEBUG: API call failed:', error);
                showAlert('An error occurred while updating profile.', 'error');
                console.error('Profile update error:', error);
            }});
        }}
        
        function resetForm() {{
            document.getElementById('first_name').value = originalData.first_name;
            document.getElementById('last_name').value = originalData.last_name;
            document.getElementById('position').value = originalData.position;
            document.getElementById('department').value = originalData.department;
            document.getElementById('phone').value = originalData.phone;
            showAlert('Form reset to original values.', 'success');
        }}
        
        // Add form change detection
        document.addEventListener('DOMContentLoaded', function() {{
            const form = document.getElementById('profileForm');
            const inputs = form.querySelectorAll('input, select');
            
            inputs.forEach(input => {{
                input.addEventListener('change', function() {{
                    // You could add visual indicators here if needed
                }});
            }});
        }});
    </script>
    """
