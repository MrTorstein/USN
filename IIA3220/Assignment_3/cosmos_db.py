"""TODOC"""

import os
from dotenv import load_dotenv

from azure.cosmos import CosmosClient

class CosmosDB():
    """
    TODOC
    """

    def __init__(self, test: bool = False):
        self.test = test

        load_dotenv()
        connection_string = os.getenv("CONNECTION_STRING")
        db_name = os.getenv("DB_NAME")
        container = os.getenv("CONTAINER")

        self.client = CosmosClient.from_connection_string(connection_string)

        database_name = os.getenv("CONFIGURATION__AZURECOSMOSDB__DATABASENAME", db_name)
        self.database = self.client.get_database_client(database_name)
        if self.test:
            print(f"Get database:\t{self.database.id}")

        container_name = os.getenv("CONFIGURATION__AZURECOSMOSDB__CONTAINERNAME", container)
        self.container = self.database.get_container_client(container_name)
        if self.test:
            print(f"Get container:\t{self.container.id}")


    def write(self, items: list) -> None:
        """TODOC"""
        if items is not None:
            for item in items:
                if item["id"]:
                    created_item = self.container.upsert_item(item)
                    if self.test:
                        print(f"Upserted item:\t{created_item}")

    def read(self, criteria: str):
        """Not Implemented Yet"""
        raise NotImplementedError

if __name__ == "__main__":
    DB_HANDLER = CosmosDB(True)
    DB_HANDLER.write([{"id":"test1", "temperature":22, "sensor":"simulator", "time":"00.01.01 00:00:00"}, {"id":"test2", "temperature":23, "sensor":"simulator", "time":"00.01.01 00:00:01"}])
