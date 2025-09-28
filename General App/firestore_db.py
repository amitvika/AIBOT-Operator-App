import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
import hashlib
import os

class FirestoreDB:
    def __init__(self, credentials_path):
        """Initialize Firestore connection with given credentials"""
        if not os.path.exists(credentials_path):
            raise FileNotFoundError(f"Credentials file not found: {credentials_path}")
        
        try:
            # Initialize Firebase with credentials
            cred = credentials.Certificate(credentials_path)
            firebase_admin.initialize_app(cred)
            
            # Get Firestore client
            self.db = firestore.client()
            print("Connected to Firestore successfully")
        except Exception as e:
            print(f"Error connecting to Firestore: {e}")
            raise
    
    def _hash_password(self, password):
        """Create a simple hash of the password.
        In production, use a proper password hashing library like bcrypt.
        """
        return hashlib.sha256(password.encode()).hexdigest()
    
    def create_operator(self, user_data):
        """Create a new operator in Firestore using an auto-generated unique ID.
        
        Args:
            user_data (dict): Dictionary containing operator details
                              (first_name, last_name, email, password, etc.)
        
        Returns:
            bool: True if successful, False otherwise.
        """
        try:
            # Hash the password before storing
            user_data["password"] = self._hash_password(user_data["password"])
            
            operators_ref = self.db.collection('operators')
            # Check if an operator with the given email already exists
            existing = operators_ref.where("email", "==", user_data["email"]).get()
            if existing:
                print("Operator with this email already exists.")
                return False
            
            # Create a new document with an auto-generated ID
            doc_ref = operators_ref.document()  # No argument means auto-generated ID
            user_data["id"] = doc_ref.id  # Save the generated ID in operator data
            doc_ref.set(user_data)
            
            return True
        except Exception as e:
            print(f"Error creating operator: {e}")
            return False
    
    def authenticate_operator(self, email, password):
        """Authenticate an operator by querying using their email.
        
        Args:
            email (str): Operator's email.
            password (str): Operator's password.
        
        Returns:
            dict: User data if authenticated, None otherwise.
        """
        try:
            operators_ref = self.db.collection('operators')
            query = operators_ref.where("email", "==", email).get()
            if not query:
                return None
            # Get the first matching document
            operator_doc = query[0]
            operator_data = operator_doc.to_dict()
            hashed_password = self._hash_password(password)
            
            if operator_data.get("password") == hashed_password:
                return operator_data
            
            return None
        except Exception as e:
            print(f"Error authenticating operator: {e}")
            return None
    
    def check_operator_exists(self, email):
        """Check if an operator with the given email exists by querying the collection.
        
        Args:
            email (str): Operator's email to check.
        
        Returns:
            bool: True if exists, False otherwise.
        """
        try:
            operators_ref = self.db.collection('operators')
            query = operators_ref.where("email", "==", email).get()
            return len(query) > 0
        except Exception as e:
            print(f"Error checking operator existence: {e}")
            return False
    
    def update_operator_status(self, email, is_online):
        """Update the online status of an operator by searching using the email field.
        
        Args:
            email (str): Operator's email.
            is_online (bool): Online status to set.
        
        Returns:
            bool: True if successful, False otherwise.
        """
        try:
            operators_ref = self.db.collection('operators')
            query = operators_ref.where("email", "==", email).get()
            if not query:
                print("Operator not found for status update")
                return False
            # Update all matching documents (should be only one)
            for doc in query:
                doc.reference.update({"online": is_online})
            return True
        except Exception as e:
            print(f"Error updating operator status: {e}")
            return False
    
    def get_all_operators(self):
        """Get a list of all operators.
        
        Returns:
            list: List of operator dictionaries.
        """
        try:
            operators = []
            operators_ref = self.db.collection('operators')
            docs = operators_ref.stream()
            for doc in docs:
                operators.append(doc.to_dict())
            return operators
        except Exception as e:
            print(f"Error getting all operators: {e}")
            return []
    
    def delete_operator(self, email):
        """Delete an operator by searching using the email field.
        
        Args:
            email (str): Email of the operator to delete.
        
        Returns:
            bool: True if successful, False otherwise.
        """
        try:
            operators_ref = self.db.collection('operators')
            query = operators_ref.where("email", "==", email).get()
            if not query:
                print("Operator not found for deletion")
                return False
            for doc in query:
                doc.reference.delete()
            return True
        except Exception as e:
            print(f"Error deleting operator: {e}")
            return False
    
    def update_task_completion(self, task_id, is_completed, completed_by=None):
        """Update the completion status of a task.
        
        Args:
            task_id (str): ID of the task to update.
            is_completed (bool): Whether the task is completed.
            completed_by (str): Email of the operator who completed the task.
        
        Returns:
            bool: True if successful, False otherwise.
        """
        try:
            task_ref = self.db.collection('tasks').document(task_id)
            update_data = {
                "is_completed": is_completed,
                "completed_at": firestore.SERVER_TIMESTAMP if is_completed else None
            }
            if completed_by:
                update_data["completed_by"] = completed_by
            task_ref.update(update_data)
            return True
        except Exception as e:
            print(f"Error updating task completion: {e}")
            return False
    
    def get_all_tasks(self):
        """Get all tasks from the tasks collection.
        
        Returns:
            list: List of task dictionaries with document IDs.
        """
        try:
            tasks = []
            tasks_ref = self.db.collection('tasks')
            docs = tasks_ref.stream()
            for doc in docs:
                task_data = doc.to_dict()
                task_data['id'] = doc.id  # Include the document ID
                tasks.append(task_data)
            return tasks
        except Exception as e:
            print(f"Error getting all tasks: {e}")
            return []
    
    def update_operator_profile(self, email, profile_data):
        """Update operator profile information.
        
        Args:
            email (str): Email of the operator to update.
            profile_data (dict): Dictionary containing profile fields to update.
        
        Returns:
            bool: True if successful, False otherwise.
        """
        try:
            print(f"DEBUG: FirestoreDB.update_operator_profile called")
            print(f"DEBUG: Email: {email}")
            print(f"DEBUG: Profile data: {profile_data}")
            
            operators_ref = self.db.collection('operators')
            query = operators_ref.where("email", "==", email).get()
            
            print(f"DEBUG: Found {len(query)} documents for email {email}")
            
            if not query:
                print("Operator not found for profile update")
                return False
            
            # Update all matching documents (should be only one)
            for doc in query:
                print(f"DEBUG: Updating document {doc.id}")
                # Don't allow updating email or password through profile update
                safe_profile_data = {k: v for k, v in profile_data.items() 
                                   if k not in ['email', 'password', 'id']}
                print(f"DEBUG: Safe profile data: {safe_profile_data}")
                
                doc.reference.update(safe_profile_data)
                print(f"DEBUG: Successfully updated document {doc.id}")
            
            return True
        except Exception as e:
            print(f"Error updating operator profile: {e}")
            import traceback
            traceback.print_exc()
            return False