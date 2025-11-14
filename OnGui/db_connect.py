

# db_connect.py
import mysql.connector

def get_connection():
    """
    MySQL DB에 접속하고 connection 객체를 반환합니다.
    신입 개발자도 이해하기 쉽도록 단순화.
    """
    connection = mysql.connector.connect(
        host="localhost",        # MySQL 서버 주소
        user="root",    # MySQL 사용자명
        password="0000",# MySQL 비밀번호
        database="patroldb" # 사용할 DB 이름
    )
    return connection

