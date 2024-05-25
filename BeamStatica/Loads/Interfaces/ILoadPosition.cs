using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BeamStatica.Loads.Interfaces
{
    public interface ILoadPosition
    {
         double Position { get; set; }
         double Value { get; set; }
    }
}
