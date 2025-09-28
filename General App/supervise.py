def get_supervise_html(db):
    """
    Generate HTML for the Supervise tab showing all tasks and their completion status.
    
    Args:
        db: An instance of FirestoreDB.
    
    Returns:
        str: HTML string representing the supervision view.
    """
    html = """
    <style>
        .supervise-container {
            padding: 20px;
        }
        .stats-section {
            display: flex;
            gap: 20px;
            margin-bottom: 30px;
        }
        .stat-card {
            background-color: #f8f9fa;
            border: 1px solid #dee2e6;
            border-radius: 8px;
            padding: 20px;
            text-align: center;
            min-width: 150px;
        }
        .stat-number {
            font-size: 2em;
            font-weight: bold;
            color: #007bff;
        }
        .stat-label {
            color: #6c757d;
            margin-top: 5px;
        }
        .tasks-table {
            width: 100%;
            border-collapse: collapse;
            background-color: white;
            border-radius: 8px;
            overflow: hidden;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .tasks-table th,
        .tasks-table td {
            padding: 12px;
            text-align: left;
            border-bottom: 1px solid #dee2e6;
        }
        .tasks-table th {
            background-color: #f8f9fa;
            font-weight: bold;
            color: #495057;
        }
        .tasks-table tr:hover {
            background-color: #f8f9fa;
        }
        .status-badge {
            padding: 4px 8px;
            border-radius: 4px;
            font-size: 12px;
            font-weight: bold;
        }
        .status-pending {
            background-color: #fff3cd;
            color: #856404;
        }
        .status-completed {
            background-color: #d4edda;
            color: #155724;
        }
        .completion-indicator {
            width: 20px;
            height: 20px;
            border-radius: 50%;
            display: inline-block;
        }
        .completion-indicator.completed {
            background-color: #28a745;
        }
        .completion-indicator.pending {
            background-color: #ffc107;
        }
    </style>
    <div class="supervise-container">
        <h2>Supervise</h2>
        <p>Monitor all tasks and their completion status across all operators.</p>
    """
    
    try:
        # Get all tasks
        tasks = db.get_all_tasks()
        
        # Calculate statistics
        total_tasks = len(tasks)
        completed_tasks = sum(1 for task in tasks if task.get("is_completed", False))
        pending_tasks = total_tasks - completed_tasks
        completion_rate = (completed_tasks / total_tasks * 100) if total_tasks > 0 else 0
        
        # Add statistics section
        html += f"""
        <div class="stats-section">
            <div class="stat-card">
                <div class="stat-number">{total_tasks}</div>
                <div class="stat-label">Total Tasks</div>
            </div>
            <div class="stat-card">
                <div class="stat-number">{completed_tasks}</div>
                <div class="stat-label">Completed</div>
            </div>
            <div class="stat-card">
                <div class="stat-number">{pending_tasks}</div>
                <div class="stat-label">Pending</div>
            </div>
            <div class="stat-card">
                <div class="stat-number">{completion_rate:.1f}%</div>
                <div class="stat-label">Completion Rate</div>
            </div>
        </div>
        """
        
        # Add tasks table
        html += """
        <h3>All Tasks</h3>
        <table class="tasks-table">
            <thead>
                <tr>
                    <th>Task</th>
                    <th>Status</th>
                    <th>Completion</th>
                    <th>Created At</th>
                    <th>Completed By</th>
                    <th>Completed At</th>
                </tr>
            </thead>
            <tbody>
        """
        
        if tasks:
            for task in tasks:
                task_id = task.get("id", "")
                description = task.get("description", "No Description")
                status = task.get("status", "Unknown")
                created_at = task.get("created_at", "Unknown")
                is_completed = task.get("is_completed", False)
                completed_by = task.get("completed_by", "")
                completed_at = task.get("completed_at", "")
                
                # Determine status badge and completion indicator
                status_class = "status-completed" if is_completed else "status-pending"
                status_text = "Completed" if is_completed else "Pending"
                completion_class = "completed" if is_completed else "pending"
                
                html += f"""
                <tr>
                    <td>{description}</td>
                    <td><span class="status-badge {status_class}">{status}</span></td>
                    <td>
                        <span class="completion-indicator {completion_class}"></span>
                        {status_text}
                    </td>
                    <td>{created_at}</td>
                    <td>{completed_by if completed_by else '-'}</td>
                    <td>{completed_at if completed_at else '-'}</td>
                </tr>
                """
        else:
            html += """
            <tr>
                <td colspan="6" style="text-align: center; padding: 20px;">
                    No tasks available.
                </td>
            </tr>
            """
        
        html += """
            </tbody>
        </table>
        """
        
    except Exception as e:
        html += f"<p>Error loading supervision data: {e}</p>"
    
    html += """
    </div>
    """
    return html
