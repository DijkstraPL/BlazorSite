﻿using KPK_CalcSite.App_Start;
using Microsoft.Owin;
using Owin;
using System.Web.Mvc;

[assembly: OwinStartupAttribute(typeof(KPK_CalcSite.Startup))]
namespace KPK_CalcSite
{
    public partial class Startup
    {
        public void Configuration(IAppBuilder app)
        {
            ConfigureAuth(app);
        }
    }
}
