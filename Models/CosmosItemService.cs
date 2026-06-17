using Microsoft.Azure.Cosmos;

namespace Air_Heater_Visualiser.Models
{
    public class CosmosItemService
    {
        private readonly CosmosClient _client;
        private readonly string _databaseName;
        private readonly string _containerName;

        public CosmosItemService(CosmosClient client, IConfiguration config)
        {
            _client = client;

            var section = config.GetSection("CosmosDb");
            _databaseName = section["DatabaseName"];
            _containerName = section["ContainerName"];
        }

        public async Task<T> GetItemAsync<T>(string id, string partitionKey)
        {
            var container = _client.GetContainer(_databaseName, _containerName);
            ItemResponse<T> response = await container.ReadItemAsync<T>(id, new PartitionKey(partitionKey));
            return response.Resource;
        }

        public async Task<List<T>> GetAllItemsAsync<T>()
        {
            var container = _client.GetContainer(_databaseName, _containerName);

            var query = new QueryDefinition("SELECT * FROM c");

            var iterator = container.GetItemQueryIterator<T>(
                query,
                requestOptions: new QueryRequestOptions
                {
                    MaxItemCount = -1,
                    PartitionKey = null
                });

            var results = new List<T>();

            while (iterator.HasMoreResults)
            {
                FeedResponse<T> response = await iterator.ReadNextAsync();
                results.AddRange(response);
            }

            return results;
        }
    }
}
