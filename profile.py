def get_profile_html(user_data):
    return f"""
    <div>
        <h2>Profile</h2>
        <p>Name: {user_data['first_name']} {user_data['last_name']}</p>
        <p>Email: {user_data['email']}</p>
        <p>Position: {user_data['position']}</p>
        <p>Account created on: {user_data['created_at']}</p>
        <!-- More profile functionalities can be added here -->
    </div>
    """
