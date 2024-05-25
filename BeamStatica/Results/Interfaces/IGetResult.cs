using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BeamStatica.Results.Interfaces
{
    public interface IGetResult
    {
        IResultValue GetValue(double distanceFromLeftSide);
    }
}
