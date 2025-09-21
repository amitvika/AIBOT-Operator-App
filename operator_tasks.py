def get_operator_tasks_html(db):
    """
    Generate HTML for the Operator Tasks tab by pulling in all tasks from the 'tasks' collection.
    Each task is displayed as a card.
    
    Args:
        db: An instance of FirestoreDB.
    
    Returns:
        str: HTML string representing the tasks.
    """
    html = """
    <style>
        .task-container {
            display: flex;
            flex-wrap: wrap;
            gap: 20px;
            padding: 20px;
        }
        .task-card {
            border: 1px solid #ddd;
            border-radius: 8px;
            padding: 15px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
            width: 250px;
            background-color: #fff;
        }
        .task-card h3 {
            margin-top: 0;
            font-size: 18px;
        }
        .task-card p {
            margin: 5px 0;
            font-size: 14px;
        }
    </style>
    <div>
        <h2>Operator Tasks</h2>
        <div class="task-container">
    """
    
    try:
        # Query all tasks from the 'tasks' collection
        tasks_ref = db.db.collection('tasks')
        tasks_docs = tasks_ref.stream()
        tasks_found = False
        for task_doc in tasks_docs:
            tasks_found = True
            task = task_doc.to_dict()
            description = task.get("description", "No Description")
            status = task.get("status", "Unknown")
            created_at = task.get("created_at", "Unknown")
            
            # Build the task card HTML
            card_html = f"""
            <div class="task-card">
                <h3>{description}</h3>
                <p><strong>Status:</strong> {status}</p>
                <p><strong>Created At:</strong> {created_at}</p>
            </div>
            """
            html += card_html
        
        if not tasks_found:
            html += "<p>No tasks available.</p>"
    except Exception as e:
        html += f"<p>Error loading tasks: {e}</p>"
    
    html += """
        </div>
    </div>
    """
    return html
