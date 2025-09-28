def get_operator_tasks_html(db):
    """
    Generate HTML for the Operator Tasks tab by pulling in all tasks from the 'tasks' collection.
    Each task is displayed as a card with completion status controls.
    
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
            width: 280px;
            background-color: #fff;
            position: relative;
        }
        .task-card.completed {
            background-color: #f0f8f0;
            border-color: #4caf50;
        }
        .task-card h3 {
            margin-top: 0;
            font-size: 18px;
        }
        .task-card p {
            margin: 5px 0;
            font-size: 14px;
        }
        .completion-status {
            display: flex;
            align-items: center;
            gap: 10px;
            margin: 10px 0;
        }
        .completion-checkbox {
            width: 18px;
            height: 18px;
            cursor: pointer;
        }
        .complete-btn {
            background-color: #4caf50;
            color: white;
            border: none;
            padding: 8px 16px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 14px;
        }
        .complete-btn:hover {
            background-color: #45a049;
        }
        .complete-btn:disabled {
            background-color: #cccccc;
            cursor: not-allowed;
        }
        .completion-info {
            font-size: 12px;
            color: #666;
            margin-top: 5px;
        }
    </style>
    <div>
        <h2>Operator Tasks</h2>
        <div class="task-container">
    """
    
    try:
        # Get all tasks using the new method
        tasks = db.get_all_tasks()
        tasks_found = len(tasks) > 0
        
        for task in tasks:
            task_id = task.get("id", "")
            description = task.get("description", "No Description")
            status = task.get("status", "Unknown")
            created_at = task.get("created_at", "Unknown")
            is_completed = task.get("is_completed", False)
            completed_by = task.get("completed_by", "")
            completed_at = task.get("completed_at", "")
            
            # Determine card class based on completion status
            card_class = "task-card completed" if is_completed else "task-card"
            
            # Build completion info
            completion_info = ""
            if is_completed:
                completion_info = f"""
                <div class="completion-info">
                    <strong>Completed by:</strong> {completed_by}<br>
                    <strong>Completed at:</strong> {completed_at}
                </div>
                """
            
            # Build the task card HTML
            card_html = f"""
            <div class="{card_class}" id="task-{task_id}">
                <h3>{description}</h3>
                <p><strong>Status:</strong> {status}</p>
                <p><strong>Created At:</strong> {created_at}</p>
                <div class="completion-status">
                    <input type="checkbox" class="completion-checkbox" id="checkbox-{task_id}" 
                           {'checked' if is_completed else ''} 
                           onchange="toggleTaskCompletion('{task_id}', this.checked)">
                    <label for="checkbox-{task_id}">Mark as Complete</label>
                </div>
                <button class="complete-btn" id="btn-{task_id}" 
                        {'disabled' if is_completed else ''}
                        onclick="markTaskComplete('{task_id}')">
                    {'Completed' if is_completed else 'Mark Complete'}
                </button>
                {completion_info}
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
    <script>
        function toggleTaskCompletion(taskId, isCompleted) {
            // This will be handled by the Python backend
            window.pywebview.api.update_task_completion(taskId, isCompleted);
        }
        
        function markTaskComplete(taskId) {
            const checkbox = document.getElementById('checkbox-' + taskId);
            const button = document.getElementById('btn-' + taskId);
            const card = document.getElementById('task-' + taskId);
            
            if (!checkbox.checked) {
                checkbox.checked = true;
                window.pywebview.api.update_task_completion(taskId, true);
                button.disabled = true;
                button.textContent = 'Completed';
                card.classList.add('completed');
            }
        }
    </script>
    """
    return html
