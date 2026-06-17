using Air_Heater_Visualiser.Models;
using Microsoft.AspNetCore.Mvc;
using Microsoft.AspNetCore.Mvc.RazorPages;

namespace Air_Heater_Visualiser.Pages
{
    public class IndexModel : PageModel
    {
        private readonly CosmosItemService _cosmos;

        public List<MyItemModel> Items { get; set; } = new();
        public List<string> Sensors { get; set; } = new();

        public IndexModel(CosmosItemService cosmos)
        {
            _cosmos = cosmos;
        }

        public async Task OnGet()
        {
            Items = await _cosmos.GetAllItemsAsync<MyItemModel>();

            Sensors = Items.Select(i => i.sensor).Distinct().ToList();
        }

        public async Task<JsonResult> OnGetChartData(string sensor)
        {
            var items = await _cosmos.GetAllItemsAsync<MyItemModel>();

            if (!string.IsNullOrEmpty(sensor))
                items = items.Where(i => i.sensor == sensor).ToList();

            items = items.OrderBy(i => i.time).ToList();

            var labels = items.Select(i => i.time).ToList();
            var temperatures = items.Select(i => i.temperature).ToList();

            return new JsonResult(new { labels, temperatures });
        }
    }

    public class MyItemModel
    {
        public string id { get; set; }
        public double temperature { get; set; }
        public string sensor { get; set; }
        public string time { get; set; }
    }
}