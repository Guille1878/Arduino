/*    ==Scripting Parameters==

    Source Server Version : SQL Server 2017 (14.0.900)
    Source Database Engine Edition : Microsoft SQL Server Enterprise Edition
    Source Database Engine Type : Standalone SQL Server

    Target Server Version : SQL Server 2017
    Target Database Engine Edition : Microsoft SQL Server Standard Edition
    Target Database Engine Type : Standalone SQL Server
*/

--USE [IntelliCasa]

/****** Object:  Table [dbo].[Casa]    Script Date: 2017-11-30 23:02:33 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
CREATE TABLE [dbo].[Casa](
	[CasaId] [uniqueidentifier] NOT NULL,
	[Name] [varchar](50) NOT NULL,
	[Updated] [datetime] NOT NULL,
	[Created] [datetime] NOT NULL,
 CONSTRAINT [PK_Casa] PRIMARY KEY CLUSTERED 
(
	[CasaId] ASC
)WITH (PAD_INDEX = OFF, STATISTICS_NORECOMPUTE = OFF, IGNORE_DUP_KEY = OFF, ALLOW_ROW_LOCKS = ON, ALLOW_PAGE_LOCKS = ON) ON [PRIMARY]
) ON [PRIMARY]
GO
/****** Object:  Table [dbo].[CollectingClientData]    Script Date: 2017-11-30 23:02:33 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
CREATE TABLE [dbo].[CollectingClientData](
	[CollectingClientDataId] [bigint] IDENTITY(1,1) NOT NULL,
	[ThingId] [uniqueidentifier] NOT NULL,
	[Value] [float] NULL,
	[Created] [datetime] NOT NULL,
 CONSTRAINT [PK_CollectingClientData] PRIMARY KEY CLUSTERED 
(
	[CollectingClientDataId] ASC
)WITH (PAD_INDEX = OFF, STATISTICS_NORECOMPUTE = OFF, IGNORE_DUP_KEY = OFF, ALLOW_ROW_LOCKS = ON, ALLOW_PAGE_LOCKS = ON) ON [PRIMARY]
) ON [PRIMARY]
GO
/****** Object:  Table [dbo].[CommandoToClient]    Script Date: 2017-11-30 23:02:33 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
CREATE TABLE [dbo].[CommandoToClient](
	[CommandoToClientId] [bigint] IDENTITY(1,1) NOT NULL,
	[ThingId] [uniqueidentifier] NOT NULL,
	[Value] [float] NOT NULL,
	[Recived] [bit] NOT NULL,
	[RecivedTime] [datetime] NULL,
	[Created] [datetime] NOT NULL,
 CONSTRAINT [PK_CommandoToClient] PRIMARY KEY CLUSTERED 
(
	[CommandoToClientId] ASC
)WITH (PAD_INDEX = OFF, STATISTICS_NORECOMPUTE = OFF, IGNORE_DUP_KEY = OFF, ALLOW_ROW_LOCKS = ON, ALLOW_PAGE_LOCKS = ON) ON [PRIMARY]
) ON [PRIMARY]
GO
/****** Object:  Table [dbo].[DataSummary]    Script Date: 2017-11-30 23:02:34 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
CREATE TABLE [dbo].[DataSummary](
	[DataSummaryId] [int] IDENTITY(1,1) NOT NULL,
	[WifiDeviceId] [uniqueidentifier] NOT NULL,
	[ThingId] [uniqueidentifier] NOT NULL,
	[Value] [float] NULL,
	[Updated] [datetime] NOT NULL,
	[Created] [datetime] NOT NULL,
 CONSTRAINT [PK_DataSummary] PRIMARY KEY CLUSTERED 
(
	[DataSummaryId] ASC
)WITH (PAD_INDEX = OFF, STATISTICS_NORECOMPUTE = OFF, IGNORE_DUP_KEY = OFF, ALLOW_ROW_LOCKS = ON, ALLOW_PAGE_LOCKS = ON) ON [PRIMARY]
) ON [PRIMARY]
GO
/****** Object:  Table [dbo].[Thing]    Script Date: 2017-11-30 23:02:34 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
CREATE TABLE [dbo].[Thing](
	[ThingId] [uniqueidentifier] NOT NULL,
	[WifiDeviceId] [uniqueidentifier] NOT NULL,
	[ProccessOrder] [smallint] NOT NULL,
	[ThingKindId] [int] NOT NULL,
	[Value] [float] NULL,
	[CanSendInformation] [bit] NOT NULL,
	[IsTrigable] [bit] NULL,
	[Updated] [datetime] NOT NULL,
	[Created] [datetime] NOT NULL,
 CONSTRAINT [PK_Thing] PRIMARY KEY CLUSTERED 
(
	[ThingId] ASC
)WITH (PAD_INDEX = OFF, STATISTICS_NORECOMPUTE = OFF, IGNORE_DUP_KEY = OFF, ALLOW_ROW_LOCKS = ON, ALLOW_PAGE_LOCKS = ON) ON [PRIMARY]
) ON [PRIMARY]
GO
/****** Object:  Table [dbo].[ThingKind]    Script Date: 2017-11-30 23:02:34 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
CREATE TABLE [dbo].[ThingKind](
	[ThingKindId] [int] IDENTITY(1,1) NOT NULL,
	[Name] [varchar](50) NOT NULL,
 CONSTRAINT [PK_ThingKind] PRIMARY KEY CLUSTERED 
(
	[ThingKindId] ASC
)WITH (PAD_INDEX = OFF, STATISTICS_NORECOMPUTE = OFF, IGNORE_DUP_KEY = OFF, ALLOW_ROW_LOCKS = ON, ALLOW_PAGE_LOCKS = ON) ON [PRIMARY]
) ON [PRIMARY]
GO
/****** Object:  Table [dbo].[WifiDevice]    Script Date: 2017-11-30 23:02:34 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
CREATE TABLE [dbo].[WifiDevice](
	[WifiDeviceId] [uniqueidentifier] NOT NULL,
	[Name] [varchar](50) NULL,
	[ShortDescription] [nvarchar](256) NULL,
	[CasaId] [uniqueidentifier] NOT NULL,
	[MacAddress] [varchar](256) NULL,
	[IPAddress] [varchar](256) NULL,
	[Updated] [datetime] NOT NULL,
	[Created] [datetime] NOT NULL,
 CONSTRAINT [PK_WifiDevice] PRIMARY KEY CLUSTERED 
(
	[WifiDeviceId] ASC
)WITH (PAD_INDEX = OFF, STATISTICS_NORECOMPUTE = OFF, IGNORE_DUP_KEY = OFF, ALLOW_ROW_LOCKS = ON, ALLOW_PAGE_LOCKS = ON) ON [PRIMARY]
) ON [PRIMARY]
GO
INSERT [dbo].[Casa] ([CasaId], [Name], [Updated], [Created]) VALUES (N'103bba7d-8236-4e26-a18f-fe4f64ca54d7', N'Esteche Fu', CAST(N'2017-11-02T21:52:07.567' AS DateTime), CAST(N'2017-11-02T21:52:07.567' AS DateTime))
GO
INSERT [dbo].[Thing] ([ThingId], [WifiDeviceId], [ProccessOrder], [ThingKindId], [Value], [CanSendInformation], [IsTrigable], [Updated], [Created]) VALUES (N'8936afff-4057-401b-9747-3af9527ffe0d', N'e698e1cc-9f01-46ca-91be-38c57b9fb1e4', 1, 3, 0, 1, 0, CAST(N'2017-11-22T22:38:34.940' AS DateTime), CAST(N'2017-11-22T22:38:34.940' AS DateTime))
GO
INSERT [dbo].[Thing] ([ThingId], [WifiDeviceId], [ProccessOrder], [ThingKindId], [Value], [CanSendInformation], [IsTrigable], [Updated], [Created]) VALUES (N'96cdbe75-f9f3-4fb8-9bec-abffe905a992', N'e698e1cc-9f01-46ca-91be-38c57b9fb1e4', 3, 1, 0, 0, 1, CAST(N'2017-11-22T22:38:13.220' AS DateTime), CAST(N'2017-11-22T22:38:13.220' AS DateTime))
GO
INSERT [dbo].[Thing] ([ThingId], [WifiDeviceId], [ProccessOrder], [ThingKindId], [Value], [CanSendInformation], [IsTrigable], [Updated], [Created]) VALUES (N'a69ab50a-f411-40e7-a4f6-caa36737e0bd', N'e698e1cc-9f01-46ca-91be-38c57b9fb1e4', 2, 4, 0, 1, 0, CAST(N'2017-11-30T22:51:12.830' AS DateTime), CAST(N'2017-11-30T22:51:12.830' AS DateTime))
GO
SET IDENTITY_INSERT [dbo].[ThingKind] ON 
GO
INSERT [dbo].[ThingKind] ([ThingKindId], [Name]) VALUES (1, N'Relay')
GO
INSERT [dbo].[ThingKind] ([ThingKindId], [Name]) VALUES (2, N'Temperature&Humidity')
GO
INSERT [dbo].[ThingKind] ([ThingKindId], [Name]) VALUES (3, N'Temperature')
GO
INSERT [dbo].[ThingKind] ([ThingKindId], [Name]) VALUES (4, N'Humidity')
GO
INSERT [dbo].[ThingKind] ([ThingKindId], [Name]) VALUES (5, N'Distance')
GO
INSERT [dbo].[ThingKind] ([ThingKindId], [Name]) VALUES (6, N'WaterLevel')
GO
INSERT [dbo].[ThingKind] ([ThingKindId], [Name]) VALUES (7, N'GroundHumidity')
GO
SET IDENTITY_INSERT [dbo].[ThingKind] OFF
GO
INSERT [dbo].[WifiDevice] ([WifiDeviceId], [Name], [ShortDescription], [CasaId], [MacAddress], [IPAddress], [Updated], [Created]) VALUES (N'e698e1cc-9f01-46ca-91be-38c57b9fb1e4', N'SwitchAndTempAndHumid', N'Balcony Hitter', N'103bba7d-8236-4e26-a18f-fe4f64ca54d7', NULL, NULL, CAST(N'2017-11-20T22:44:12.380' AS DateTime), CAST(N'2017-11-20T22:44:12.380' AS DateTime))
GO
ALTER TABLE [dbo].[Casa] ADD  CONSTRAINT [DF_Casa_CasaId]  DEFAULT (newid()) FOR [CasaId]
GO
ALTER TABLE [dbo].[Casa] ADD  CONSTRAINT [DF_Casa_Name]  DEFAULT ('') FOR [Name]
GO
ALTER TABLE [dbo].[Casa] ADD  CONSTRAINT [DF_Casa_Updated]  DEFAULT (getdate()) FOR [Updated]
GO
ALTER TABLE [dbo].[Casa] ADD  CONSTRAINT [DF_Casa_Created]  DEFAULT (getdate()) FOR [Created]
GO
ALTER TABLE [dbo].[CollectingClientData] ADD  CONSTRAINT [DF_CollectingClientData_Created]  DEFAULT (getdate()) FOR [Created]
GO
ALTER TABLE [dbo].[CommandoToClient] ADD  CONSTRAINT [DF_CommandoToClient_Recived]  DEFAULT ((0)) FOR [Recived]
GO
ALTER TABLE [dbo].[CommandoToClient] ADD  CONSTRAINT [DF_CommandoToClient_Created]  DEFAULT (getdate()) FOR [Created]
GO
ALTER TABLE [dbo].[DataSummary] ADD  CONSTRAINT [DF_DataSummary_Updated]  DEFAULT (getdate()) FOR [Updated]
GO
ALTER TABLE [dbo].[DataSummary] ADD  CONSTRAINT [DF_DataSummary_Created]  DEFAULT (getdate()) FOR [Created]
GO
ALTER TABLE [dbo].[Thing] ADD  CONSTRAINT [DF_Thing_ThingId]  DEFAULT (newid()) FOR [ThingId]
GO
ALTER TABLE [dbo].[Thing] ADD  CONSTRAINT [DF_Thing_ProccessOrder]  DEFAULT ((1)) FOR [ProccessOrder]
GO
ALTER TABLE [dbo].[Thing] ADD  CONSTRAINT [DF_Thing_CanSendInformation]  DEFAULT ((0)) FOR [CanSendInformation]
GO
ALTER TABLE [dbo].[Thing] ADD  CONSTRAINT [DF_Thing_IsTrigable]  DEFAULT ((0)) FOR [IsTrigable]
GO
ALTER TABLE [dbo].[Thing] ADD  CONSTRAINT [DF_Thing_Updated]  DEFAULT (getdate()) FOR [Updated]
GO
ALTER TABLE [dbo].[Thing] ADD  CONSTRAINT [DF_Thing_Created]  DEFAULT (getdate()) FOR [Created]
GO
ALTER TABLE [dbo].[WifiDevice] ADD  CONSTRAINT [DF_WifiDevice_ClientDeviceId]  DEFAULT (newid()) FOR [WifiDeviceId]
GO
ALTER TABLE [dbo].[WifiDevice] ADD  CONSTRAINT [DF_WifiDevice_Updated]  DEFAULT (getdate()) FOR [Updated]
GO
ALTER TABLE [dbo].[WifiDevice] ADD  CONSTRAINT [DF_WifiDevice_Created]  DEFAULT (getdate()) FOR [Created]
GO
ALTER TABLE [dbo].[Thing]  WITH CHECK ADD  CONSTRAINT [FK_Thing_Thing] FOREIGN KEY([ThingId])
REFERENCES [dbo].[Thing] ([ThingId])
GO
ALTER TABLE [dbo].[Thing] CHECK CONSTRAINT [FK_Thing_Thing]
GO
ALTER TABLE [dbo].[Thing]  WITH CHECK ADD  CONSTRAINT [FK_Thing_ThingKind] FOREIGN KEY([ThingKindId])
REFERENCES [dbo].[ThingKind] ([ThingKindId])
GO
ALTER TABLE [dbo].[Thing] CHECK CONSTRAINT [FK_Thing_ThingKind]
GO
ALTER TABLE [dbo].[Thing]  WITH CHECK ADD  CONSTRAINT [FK_Thing_WifiDevice] FOREIGN KEY([WifiDeviceId])
REFERENCES [dbo].[WifiDevice] ([WifiDeviceId])
GO
ALTER TABLE [dbo].[Thing] CHECK CONSTRAINT [FK_Thing_WifiDevice]
GO
ALTER TABLE [dbo].[ThingKind]  WITH CHECK ADD  CONSTRAINT [FK_ThingKind_ThingKind] FOREIGN KEY([ThingKindId])
REFERENCES [dbo].[ThingKind] ([ThingKindId])
GO
ALTER TABLE [dbo].[ThingKind] CHECK CONSTRAINT [FK_ThingKind_ThingKind]
GO
ALTER TABLE [dbo].[WifiDevice]  WITH CHECK ADD  CONSTRAINT [FK_WifiDevice_Casa] FOREIGN KEY([CasaId])
REFERENCES [dbo].[Casa] ([CasaId])
GO
ALTER TABLE [dbo].[WifiDevice] CHECK CONSTRAINT [FK_WifiDevice_Casa]
GO
/****** Object:  StoredProcedure [dbo].[sp_Casa_GetAll]    Script Date: 2017-11-30 23:02:35 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
-- =============================================
-- Author:		<Author,,Name>
-- Create date: <Create Date,,>
-- Description:	<Description,,>
-- =============================================
CREATE PROCEDURE [dbo].[sp_Casa_GetAll]

AS
BEGIN
	-- SET NOCOUNT ON added to prevent extra result sets from
	-- interfering with SELECT statements.
	SET NOCOUNT ON;

	SELECT TOP (100000) [CasaId]
      ,[Name]
      ,[Updated]
      ,[Created]
  FROM [dbo].[Casa]

END
GO
/****** Object:  StoredProcedure [dbo].[sp_ClientDevice_GetCurrentInfo]    Script Date: 2017-11-30 23:02:35 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
-- =============================================
-- Author:		<Author,,Name>
-- Create date: <Create Date,,>
-- Description:	<Description,,>
-- =============================================
CREATE PROCEDURE [dbo].[sp_ClientDevice_GetCurrentInfo]
	-- Add the parameters for the stored procedure here
	@ClientDeviceId uniqueidentifier
AS
BEGIN
	-- SET NOCOUNT ON added to prevent extra result sets from
	-- interfering with SELECT statements.
	SET NOCOUNT ON;

	Declare @temperatur float = NULL,
			@huminity float = NULL,
			@distance float = NULL,
			@commandoWaiting int = 0,
			@OnOrOff bit = NULL,
			@porcentDimmer smallint = NULL

	Select @temperatur = ccd.Temperatur
	From dbo.CollectingClientData ccd
	Where @ClientDeviceId = ccd.ClientDeviceId
	  And ccd.Temperatur is not null
	Order by ccd.Created desc

	Select Top 1 @huminity = ccd.Huminity
	From dbo.CollectingClientData ccd
	Where @ClientDeviceId = ccd.ClientDeviceId
	And ccd.Huminity is not null
	Order by ccd.Created desc

	Select Top 1 @distance = ccd.Distance
	From dbo.CollectingClientData ccd
	Where @ClientDeviceId = ccd.ClientDeviceId
	And ccd.Distance is not null
	Order by ccd.Created desc

	Select @commandoWaiting = Count(ctc.CommandoToClientId)
	From dbo.CommandoToClient ctc 
	Where ctc.ClientDeviceId = @ClientDeviceId
	  And ctc.Recived = 0

	Select Top 1 @OnOrOff = Case When ctc.TurnOn = 1 Then 1 When ctc.TurnOff = 1 Then 0 Else NULL End
	From dbo.CommandoToClient ctc 
	Where ctc.ClientDeviceId = @ClientDeviceId
	  And ctc.Recived = 1
	Order by ctc.Created desc

	Select Top 1 @porcentDimmer = ctc.PorcentDimming
	From dbo.CommandoToClient ctc 
	Where ctc.ClientDeviceId = @ClientDeviceId
	  And ctc.Recived = 1
	  And ctc.PorcentDimming is not null	  
	Order by ctc.Created desc

    SELECT cd.[ClientDeviceId]
      ,cd.[Name] as ClientName
      ,[ShortDescription]
      ,cd.[CasaId]
	  ,c.Name as CasaName
      ,[MacAddress]
      ,[IPAddress]
	  ,@temperatur as Temperatur
	  ,@huminity as Huminity
	  ,@distance as Distance
	  ,@porcentDimmer as PorcentDimmer
	  ,@OnOrOff as OnOrOff
	  ,@commandoWaiting as CommandoWaiting
	FROM [dbo].[ClientDevice] cd
	Join dbo.Casa c On cd.CasaId = c.CasaId
	

END
GO
/****** Object:  StoredProcedure [dbo].[sp_CollectingClientData_Add]    Script Date: 2017-11-30 23:02:35 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
-- =============================================
-- Author:		<Author,,Name>
-- Create date: <Create Date,,>
-- Description:	<Description,,>
-- =============================================
CREATE PROCEDURE [dbo].[sp_CollectingClientData_Add]
	
	@ThingId uniqueidentifier,
	@Value float 

AS
BEGIN

Declare @DataSummaryId int = NULL,
		@CurrentValue float = NULL

SELECT @DataSummaryId = [DataSummaryId], @CurrentValue = [Value]
FROM [dbo].[DataSummary] 
Where [ThingId] = @ThingId

If (@DataSummaryId Is not null)
 Begin

	if (@CurrentValue <> @Value)
	 Begin
		INSERT INTO [dbo].[CollectingClientData]
			   ([ThingId],[Value])
		 VALUES
			   (@ThingId,@Value)

		UPDATE [dbo].[DataSummary]
		SET [Value] = @Value
		  ,[Updated] = Getdate()
		Where [ThingId] = @ThingId

	 End
 End
Else
 Begin

	Declare @WifiDeviceId uniqueidentifier

	INSERT INTO [dbo].[CollectingClientData]
			   ([ThingId],[Value])
		 VALUES
			   (@ThingId,@Value)

	SELECT TOP (1) @WifiDeviceId = [WifiDeviceId]
	FROM [dbo].[Thing]
	Where [ThingId] = @ThingId

	INSERT INTO [dbo].[DataSummary]
           ([WifiDeviceId],[ThingId],[Value])
     VALUES
           (@WifiDeviceId,@ThingId,@Value)

 End

END
GO
/****** Object:  StoredProcedure [dbo].[sp_CommandoToClient_AddNew]    Script Date: 2017-11-30 23:02:35 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
-- =============================================
-- Author:		<Author,,Name>
-- Create date: <Create Date,,>
-- Description:	<Description,,>
-- =============================================
CREATE PROCEDURE [dbo].[sp_CommandoToClient_AddNew]
	@ThingId uniqueidentifier,
	@Value float,
	@CommandoId_OUT bigint OUTPUT
AS
BEGIN
	
	INSERT INTO [dbo].[CommandoToClient]
           ([ThingId]
           ,[Value])
     VALUES
           (@ThingId
           ,@Value)

	set @CommandoId_OUT = SCOPE_IDENTITY()

END
GO
/****** Object:  StoredProcedure [dbo].[sp_CommandoToClient_GetAllNotRecived]    Script Date: 2017-11-30 23:02:35 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
-- =============================================
-- Author:		<Author,,Name>
-- Create date: <Create Date,,>
-- Description:	<Description,,>
-- =============================================
Create PROCEDURE [dbo].[sp_CommandoToClient_GetAllNotRecived]
	
AS
BEGIN
	
	SELECT TOP (1000) [CommandoToClientId]
      ,[ThingId]
      ,[Value]
      ,[Recived]
      ,[Created]
  FROM [dbo].[CommandoToClient]
  Where [Recived] = 0

	

END
GO
/****** Object:  StoredProcedure [dbo].[sp_CommandoToClient_UpdateRecived]    Script Date: 2017-11-30 23:02:35 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
-- =============================================
-- Author:		<Author,,Name>
-- Create date: <Create Date,,>
-- Description:	<Description,,>
-- =============================================
CREATE PROCEDURE [dbo].[sp_CommandoToClient_UpdateRecived]
	@CommandoId bigint
AS
BEGIN
	
	Update [dbo].[CommandoToClient]
	Set [Recived] = 1,
		[RecivedTime] = Getdate()
	Where  CommandoToClientId = @CommandoId
	
END
GO
/****** Object:  StoredProcedure [dbo].[sp_Thing_GetByDeviceId]    Script Date: 2017-11-30 23:02:35 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
-- =============================================
-- Author:		<Author,,Name>
-- Create date: <Create Date,,>
-- Description:	<Description,,>
-- =============================================
CREATE PROCEDURE [dbo].[sp_Thing_GetByDeviceId]
	-- Add the parameters for the stored procedure here
	@WifiDeviceId uniqueidentifier
AS
BEGIN
	
	SET NOCOUNT ON;

    SELECT TOP (1000) [ThingId]
      ,[WifiDeviceId]
      ,[ProccessOrder]
      ,[ThingKindId]
      ,[Value]
      ,[CanSendInformation]
	  ,[IsTrigable]
      ,[Updated]
      ,[Created]
  FROM [dbo].[Thing]
  Where [WifiDeviceId] = @WifiDeviceId

END
GO
/****** Object:  StoredProcedure [dbo].[sp_ThingKind_GetAll]    Script Date: 2017-11-30 23:02:35 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
-- =============================================
-- Author:		<Author,,Name>
-- Create date: <Create Date,,>
-- Description:	<Description,,>
-- =============================================
CREATE PROCEDURE [dbo].[sp_ThingKind_GetAll]
	
AS
BEGIN
	-- SET NOCOUNT ON added to prevent extra result sets from
	-- interfering with SELECT statements.
	SET NOCOUNT ON;

    SELECT TOP (1000) [ThingKindId]
      ,[Name]
  FROM [dbo].[ThingKind]

END
GO
/****** Object:  StoredProcedure [dbo].[sp_WifiDevice_GetByCasaId]    Script Date: 2017-11-30 23:02:35 ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
-- =============================================
-- Author:		<Author,,Name>
-- Create date: <Create Date,,>
-- Description:	<Description,,>
-- =============================================
CREATE PROCEDURE [dbo].[sp_WifiDevice_GetByCasaId]
	@CasaId uniqueidentifier
AS
BEGIN
	
	SET NOCOUNT ON;

    SELECT TOP (1000) [WifiDeviceId]
      ,[Name]
      ,[ShortDescription]
      ,[CasaId]
      ,[MacAddress]
      ,[IPAddress]
      ,[Updated]
      ,[Created]
  FROM [dbo].[WifiDevice]
  Where [CasaId] = @CasaId

END
GO
